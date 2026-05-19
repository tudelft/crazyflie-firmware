#!/usr/bin/env python3
"""
Evaluate RL controller logs against a Python re-implementation of the firmware
observation pipeline and the neural network.

What it does:
  1. Parses weights/biases from examples/app_rl_controller/src/neural_net.c
  2. Loads a flight log CSV
  3. Reconstructs the 21-element observation vector the same way
     rl_controller.c::computeObservation() does
  4. Runs the MLP forward pass in numpy
  5. Compares the recomputed NN output to the logged rlapp.nnOut*
  6. Flags anomalies — frames, signs, out-of-distribution inputs, etc.

If the recomputed NN output closely matches the logged output across a whole
flight, our Python reconstruction is faithful to the firmware and we can trust
the diagnostic plots.  Large mismatches isolate where a sign/frame is wrong.

Usage:
    python eval_rl.py log24.csv
    python eval_rl.py log24.csv --start-row 2400 --end-row 4000
"""

import argparse
import os
import re
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Constants mirroring rl_controller.c
# ---------------------------------------------------------------------------
NUM_GATES = 8
GATE_R = 1.5
DEFAULT_GATE_ORIGIN = (0.0, 0.0)
DEFAULT_GATE_ALT = 1.5  # ENU; NED z = -1.5

BASE_GATE_X = np.array([ GATE_R,  0.0, -GATE_R,  0.0,  GATE_R,  0.0, -GATE_R,  0.0])
BASE_GATE_Y = np.array([-GATE_R,  0.0,  GATE_R,  3.0,  GATE_R,  0.0, -GATE_R, -3.0])
BASE_GATE_YAW = np.array([
    np.pi/2,  np.pi,   np.pi/2,  0.0,
   -np.pi/2, -np.pi,  -np.pi/2,  0.0,
])

INPUT_DIM = 21
OUTPUT_DIM = 4
HIDDEN = 96


# ---------------------------------------------------------------------------
# Weight extraction
# ---------------------------------------------------------------------------
def parse_c_array(src: str, name: str) -> np.ndarray:
    """Parse `static const float <name>[N] = { ... };` from C source."""
    m = re.search(
        rf"static\s+const\s+float\s+{re.escape(name)}\s*\[\s*(\d+)\s*\]\s*=\s*\{{([^}}]*)\}}",
        src, flags=re.DOTALL,
    )
    if not m:
        raise RuntimeError(f"Could not find C array `{name}`")
    expected = int(m.group(1))
    body = m.group(2)
    # Strip trailing `f` from float literals
    vals = [float(tok.rstrip("f")) for tok in re.findall(r"-?\d+\.\d+(?:[eE][+-]?\d+)?f?", body)]
    if len(vals) != expected:
        raise RuntimeError(f"{name}: expected {expected} values, got {len(vals)}")
    return np.asarray(vals, dtype=np.float32)


def load_network(c_path: str):
    with open(c_path, "r") as f:
        src = f.read()
    w0 = parse_c_array(src, "layer0_weights").reshape(HIDDEN, INPUT_DIM)
    b0 = parse_c_array(src, "layer0_biases")
    w1 = parse_c_array(src, "layer1_weights").reshape(HIDDEN, HIDDEN)
    b1 = parse_c_array(src, "layer1_biases")
    w2 = parse_c_array(src, "layer2_weights").reshape(OUTPUT_DIM, HIDDEN)
    b2 = parse_c_array(src, "layer2_biases")
    return (w0, b0, w1, b1, w2, b2)


def forward(obs: np.ndarray, net) -> np.ndarray:
    w0, b0, w1, b1, w2, b2 = net
    h0 = np.tanh(w0 @ obs + b0)
    h1 = np.tanh(w1 @ h0 + b1)
    out = w2 @ h1 + b2
    return np.clip(out, -1.0, 1.0)


# ---------------------------------------------------------------------------
# Gate layout
# ---------------------------------------------------------------------------
def init_gates(origin=DEFAULT_GATE_ORIGIN, altitude=DEFAULT_GATE_ALT):
    gx = BASE_GATE_X + origin[0]
    gy = BASE_GATE_Y + origin[1]
    gz = np.full(NUM_GATES, -altitude)  # NED: z negative = up
    gyaw = BASE_GATE_YAW.copy()
    return gx, gy, gz, gyaw


# ---------------------------------------------------------------------------
# Observation reconstruction (mirrors rl_controller.c::computeObservation)
# ---------------------------------------------------------------------------
def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def euler_to_quat(phi, theta, psi):
    """ZYX Euler → quaternion (wxyz), matches eulerToQuat in rl_controller.c"""
    cp, sp = np.cos(phi*0.5), np.sin(phi*0.5)
    ct, st = np.cos(theta*0.5), np.sin(theta*0.5)
    cy, sy = np.cos(psi*0.5), np.sin(psi*0.5)
    qw = cp*ct*cy + sp*st*sy
    qx = sp*ct*cy - cp*st*sy
    qy = cp*st*cy + sp*ct*sy
    qz = cp*ct*sy - sp*st*cy
    return qw, qx, qy, qz


def world_to_body_vel(vx, vy, vz, roll, pitch, yaw):
    """Rotate world-frame velocity (firmware ENU) into CF body frame.

    Firmware body frame:  x forward, y left, z up (roll/pitch/yaw ZYX).
    The firmware actually reads kalman.statePX/PY/PZ directly, but we don't
    have those in the log — so we reconstruct from world velocity + attitude.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # R_body_to_world = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    # body = R^T @ world
    R_wb = np.array([
        [ cy*cp, sy*cp, -sp    ],
        [ cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
        [ cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr],
    ])
    v = np.stack([vx, vy, vz], axis=-1)
    return v @ R_wb.T


def compute_observation(fw_x, fw_y, fw_z,
                        fw_bvx, fw_bvy, fw_bvz,
                        fw_roll_deg, fw_pitch_deg, fw_yaw_deg,
                        fw_gx_dps, fw_gy_dps, fw_gz_dps,
                        target_gate, last_actions,
                        gx, gy, gz, gyaw):
    """Reproduce rl_controller.c::computeObservation exactly."""
    # ENU → sim NED-like
    sim_x =  fw_x
    sim_y = -fw_y
    sim_z = -fw_z

    phi   =  np.deg2rad(fw_roll_deg)
    theta =  np.deg2rad(fw_pitch_deg)
    psi   = -np.deg2rad(fw_yaw_deg)

    # CF body (fwd,left,up) → sim body (fwd,right,down)
    u = fw_bvx
    v = -fw_bvy
    w = -fw_bvz

    p =  np.deg2rad(fw_gx_dps)
    q = -np.deg2rad(fw_gy_dps)
    r = -np.deg2rad(fw_gz_dps)

    qw, qx, qy, qz = euler_to_quat(phi, theta, psi)

    gi = target_gate % NUM_GATES
    nxt = (target_gate + 1) % NUM_GATES

    obs = np.zeros(INPUT_DIM, dtype=np.float32)
    # [0-2] current-gate position (world NED)
    obs[0] = gx[gi] - sim_x
    obs[1] = gy[gi] - sim_y
    obs[2] = gz[gi] - sim_z
    # [3-5] body velocity (sim NED body)
    obs[3] = u
    obs[4] = v
    obs[5] = w
    # [6-9] gate-relative quaternion (yaw-only rotation removed)
    cg = np.cos(gyaw[gi] * 0.5)
    sg = np.sin(gyaw[gi] * 0.5)
    qw_r = cg*qw + sg*qz
    qx_r = cg*qx + sg*qy
    qy_r = cg*qy - sg*qx
    qz_r = cg*qz - sg*qw
    n = np.sqrt(qw_r*qw_r + qx_r*qx_r + qy_r*qy_r + qz_r*qz_r)
    if n < 1e-8: n = 1e-8
    obs[6] = qw_r / n
    obs[7] = qx_r / n
    obs[8] = qy_r / n
    obs[9] = qz_r / n
    # [10-12] angular rates (sim body)
    obs[10] = p
    obs[11] = q
    obs[12] = r
    # [13-16] previous actions
    obs[13:17] = last_actions
    # [17-19] next gate position (world NED)
    obs[17] = gx[nxt] - sim_x
    obs[18] = gy[nxt] - sim_y
    obs[19] = gz[nxt] - sim_z
    # [20] next gate relative yaw
    drone_yaw = np.arctan2(2.0 * (qw*qz + qx*qy),
                           1.0 - 2.0 * (qy*qy + qz*qz))
    raw = gyaw[nxt] - drone_yaw + np.pi
    obs[20] = ((raw % (2*np.pi)) + 2*np.pi) % (2*np.pi) - np.pi
    return obs


# ---------------------------------------------------------------------------
# Gate-passing detection (mirrors checkGatePassing)
# ---------------------------------------------------------------------------
def check_gate_pass(prev_sim_x, prev_sim_y, sim_x, sim_y,
                    target_gate, gx, gy, gyaw):
    gi = target_gate % NUM_GATES
    nx = np.cos(gyaw[gi])
    ny = np.sin(gyaw[gi])
    old_proj = (prev_sim_x - gx[gi]) * nx + (prev_sim_y - gy[gi]) * ny
    new_proj = (sim_x      - gx[gi]) * nx + (sim_y      - gy[gi]) * ny
    if old_proj < 0.0 and new_proj >= 0.0:
        return (target_gate + 1) % NUM_GATES, True
    return target_gate, False


# ---------------------------------------------------------------------------
# RL-start detection: find frame where the NN first drives the motors.
# During hovering, the PID runs the motors and the NN output is computed
# purely for logging; motor override kicks in at STATE_RL_CONTROL.  We detect
# this via sudden saturation of m2/m4 (PWM_MAX = 60000) which matches the NN
# action scaling — rare in the PID hover.
# ---------------------------------------------------------------------------
def detect_rl_start(df, saturation_threshold=55000):
    """Return the first index where both flap motors are near the NN PWM max."""
    sat = (df["motor.m2"] > saturation_threshold) | (df["motor.m4"] > saturation_threshold)
    idx = np.where(sat.values)[0]
    return idx[0] if len(idx) else None


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------
def analyse(df, net, origin, altitude, rl_start=None):
    """Reconstruct the firmware's observation and compare NN output.

    The log samples the state at ~500 Hz, but the RL task only runs the
    network at ~200 Hz.  Between task ticks, rlapp.nnOut* is a *cached*
    value from the last tick.  We detect RL ticks as rows where nnOut
    actually changes from the previous row, and only run Python forward
    at those rows — otherwise we'd be comparing firmware's stale cached
    output to a network evaluated on fresh sensor data.
    """
    gx, gy, gz, gyaw = init_gates(origin, altitude)

    # Detect "tick rows": rows where the logged nnOut has changed vs prev row.
    # These are the frames at which the firmware actually ran the NN.
    nn_cols = ["rlapp.nnOut0", "rlapp.nnOut1", "rlapp.nnOut2", "rlapp.nnOut3"]
    nn_logged_full = df[nn_cols].to_numpy(dtype=np.float32)
    changed = np.any(np.abs(np.diff(nn_logged_full, axis=0)) > 1e-7, axis=1)
    # First row + any row where value changed vs previous
    tick_mask = np.concatenate([[True], changed])
    tick_rows = np.where(tick_mask)[0]

    # Running state (updated only at tick rows)
    target_gate = 0
    last_actions = np.zeros(4, dtype=np.float32)
    prev_sim_x =  df["stateEstimate.x"].iloc[0]
    prev_sim_y = -df["stateEstimate.y"].iloc[0]

    # Body velocity: prefer kalman.statePX/PY/PZ (what the firmware actually
    # reads). Fall back to rotating world ENU velocity into body if not logged.
    if {"kalman.statePX", "kalman.statePY", "kalman.statePZ"}.issubset(df.columns):
        body_vel = np.stack([
            df["kalman.statePX"].to_numpy(dtype=np.float32),
            df["kalman.statePY"].to_numpy(dtype=np.float32),
            df["kalman.statePZ"].to_numpy(dtype=np.float32),
        ], axis=-1)
    else:
        roll_r  = np.deg2rad(df["stateEstimate.roll"].to_numpy())
        pitch_r = np.deg2rad(df["stateEstimate.pitch"].to_numpy())
        yaw_r   = np.deg2rad(df["stateEstimate.yaw"].to_numpy())
        body_vel = np.empty((len(df), 3), dtype=np.float32)
        for i in range(len(df)):
            body_vel[i] = world_to_body_vel(
                df["stateEstimate.vx"].iloc[i],
                df["stateEstimate.vy"].iloc[i],
                df["stateEstimate.vz"].iloc[i],
                roll_r[i], pitch_r[i], yaw_r[i])

    N = len(df)
    obs_all = np.zeros((N, INPUT_DIM), dtype=np.float32)
    nn_recomputed = np.zeros((N, OUTPUT_DIM), dtype=np.float32)
    tgt_gate = np.zeros(N, dtype=np.int32)
    gate_passes = []

    # Walk every row for prev-position tracking (gate detection is per-row)
    # but only run forward at tick rows
    for i in range(N):
        sim_x =  df["stateEstimate.x"].iloc[i]
        sim_y = -df["stateEstimate.y"].iloc[i]
        if i > 0:
            new_tg, passed = check_gate_pass(
                prev_sim_x, prev_sim_y, sim_x, sim_y,
                target_gate, gx, gy, gyaw)
            if passed:
                gate_passes.append((i, target_gate, new_tg))
            target_gate = new_tg
        tgt_gate[i] = target_gate
        prev_sim_x, prev_sim_y = sim_x, sim_y

        if tick_mask[i]:
            # Firmware ran NN at this row — reconstruct and compare
            obs = compute_observation(
                df["stateEstimate.x"].iloc[i],
                df["stateEstimate.y"].iloc[i],
                df["stateEstimate.z"].iloc[i],
                body_vel[i, 0], body_vel[i, 1], body_vel[i, 2],
                df["stateEstimate.roll"].iloc[i],
                df["stateEstimate.pitch"].iloc[i],
                df["stateEstimate.yaw"].iloc[i],
                df["gyro.x"].iloc[i],
                df["gyro.y"].iloc[i],
                df["gyro.z"].iloc[i],
                target_gate,
                last_actions,
                gx, gy, gz, gyaw,
            )
            obs_all[i] = obs
            nn_recomputed[i] = forward(obs, net)
            # Update last_actions = what the firmware stored after *its* forward,
            # which is this row's logged output
            last_actions = nn_logged_full[i]
        else:
            # Forward-fill: carry the last computed obs/nn_reco for plotting
            obs_all[i] = obs_all[i - 1] if i > 0 else np.zeros(INPUT_DIM)
            nn_recomputed[i] = nn_recomputed[i - 1] if i > 0 else np.zeros(OUTPUT_DIM)

    # Difference: logged minus recomputed (only meaningful at tick rows)
    diff = nn_logged_full - nn_recomputed

    return dict(
        obs=obs_all, nn_recomputed=nn_recomputed, nn_logged=nn_logged_full,
        diff=diff, body_vel=body_vel, tgt_gate=tgt_gate,
        gates=(gx, gy, gz, gyaw), gate_passes=gate_passes,
        tick_mask=tick_mask, tick_rows=tick_rows,
    )


# ---------------------------------------------------------------------------
# Anomaly reporting
# ---------------------------------------------------------------------------
def report_anomalies(df, res, rl_start):
    print("\n" + "=" * 72)
    print("ANOMALY REPORT")
    print("=" * 72)

    N = len(df)
    scope_lo = rl_start if rl_start is not None else 0
    # Only evaluate at tick rows (frames where firmware ran the NN)
    tick_mask = res["tick_mask"]
    scope_tick = tick_mask.copy()
    scope_tick[:scope_lo] = False

    n_ticks_total = int(tick_mask.sum())
    n_ticks_scope = int(scope_tick.sum())
    print(f"\n  Tick rows (where firmware ran NN): {n_ticks_total} / {N} log rows "
          f"(~{n_ticks_total/N*100:.0f}%)")
    print(f"  Ticks in analysis scope: {n_ticks_scope}")

    # 1. Match between logged and recomputed NN output
    diff = res["diff"][scope_tick]
    if len(diff):
        rms = np.sqrt(np.mean(diff ** 2, axis=0))
        peak = np.max(np.abs(diff), axis=0)
    else:
        rms = peak = np.zeros(OUTPUT_DIM)
    print("\n[1] Logged vs recomputed NN output (tick rows, RL phase):")
    for k in range(OUTPUT_DIM):
        print(f"     nnOut{k}: RMS={rms[k]:.4f}  peak={peak[k]:.4f}")
    print("    ↳ RMS < ~0.05 means Python reconstruction matches firmware.")
    print("      Larger values = sign/frame bug in observation OR body-velocity")
    print("      approximation error (log lacks kalman.statePX/PY/PZ).")

    # 2. Per-step obs stats (at tick rows, in scope)
    obs = res["obs"][scope_tick]
    print("\n[2] Observation range during RL phase (expect |x| mostly < 5):")
    labels = [
        "gate.x-drone.x", "gate.y-drone.y", "gate.z-drone.z",
        "u_body(fwd)", "v_body(right)", "w_body(down)",
        "q_rel_w", "q_rel_x", "q_rel_y", "q_rel_z",
        "p", "q(sim)", "r(sim)",
        "lastAct0", "lastAct1", "lastAct2", "lastAct3",
        "next.x-drone.x", "next.y-drone.y", "next.z-drone.z",
        "next_yaw_rel",
    ]
    for i in range(INPUT_DIM):
        lo, hi = np.min(obs[:, i]), np.max(obs[:, i])
        mean = np.mean(obs[:, i])
        flag = "  <-- out of range" if max(abs(lo), abs(hi)) > 10 else ""
        print(f"     [{i:2d}] {labels[i]:<20s}  min={lo:+.2f}  max={hi:+.2f}  mean={mean:+.2f}{flag}")

    # 3. Sanity: at rl_start, does gate-to-drone point to gate 0?
    if rl_start is not None:
        print("\n[3] Frame sanity checks at RL start (row {}):".format(rl_start))
        gx, gy, gz, _ = res["gates"]
        fw_x = df["stateEstimate.x"].iloc[rl_start]
        fw_y = df["stateEstimate.y"].iloc[rl_start]
        fw_z = df["stateEstimate.z"].iloc[rl_start]
        sim_x, sim_y, sim_z = fw_x, -fw_y, -fw_z
        print(f"     Firmware ENU position:  x={fw_x:+.2f}  y={fw_y:+.2f}  z={fw_z:+.2f}")
        print(f"     Sim NED position:       x={sim_x:+.2f}  y={sim_y:+.2f}  z={sim_z:+.2f}")
        print(f"     Gate 0 NED position:    x={gx[0]:+.2f}  y={gy[0]:+.2f}  z={gz[0]:+.2f}")
        print(f"     gate0 − drone:          "
              f"x={gx[0]-sim_x:+.2f}  y={gy[0]-sim_y:+.2f}  z={gz[0]-sim_z:+.2f}")
        # Gate 0 normal points in +NED-y.  Drone must approach from y < -1.5.
        approach_proj = sim_y - gy[0]
        side = "BEHIND (good)" if approach_proj < 0 else "ALREADY PAST (bad - will not detect crossing)"
        print(f"     Drone is {side} gate 0 plane (proj={approach_proj:+.2f})")

    # 4. Gate passing events
    print("\n[4] Inferred gate crossings:")
    if not res["gate_passes"]:
        print("     (none — drone never crossed a gate plane)")
    else:
        for i, from_g, to_g in res["gate_passes"]:
            t_ms = df["timestamp"].iloc[i] - df["timestamp"].iloc[0]
            print(f"     row={i}  t={t_ms/1000:.2f}s  gate {from_g} → {to_g}")

    # 5. Key limitations & next steps
    missing = []
    for col in ["kalman.statePX", "kalman.statePY", "kalman.statePZ"]:
        if col not in df.columns:
            missing.append(col)
    print("\n[5] Known limitations of this analysis:")
    if missing:
        print("    • " + ", ".join(missing) + " are NOT in the log.")
        print("      We reconstruct body velocity by rotating stateEstimate.v{x,y,z}")
        print("      (world frame) into the body frame with the logged attitude.")
        print("      The firmware actually reads kalman.stateP{X,Y,Z} directly, so")
        print("      small differences here will drive nonzero RMS even if the")
        print("      frame conventions in the C code are correct.")
        print("      ⇒ For a definitive test, re-fly with these added to usdlog")
        print("        config and re-run this script.")
    else:
        print("    • kalman.stateP* is in the log — body velocity is authoritative.")
    print("    • The firmware also doesn't log rlapp.tgtGate or rlapp.state. Gate")
    print("      indices are inferred by replaying the plane-crossing logic here.")
    print("      Adding LOG_ADD entries for those would eliminate any ambiguity.")
    print("    • Even better: add temporary LOG_ADD entries for the observation")
    print("      vector itself (obs[0..20]) during a test flight — then the")
    print("      firmware's observation can be diff-ed exactly against Python's.")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_diagnostics(df, res, rl_start, title=""):
    gx, gy, _, gyaw = res["gates"]
    nn_logged = res["nn_logged"]
    nn_reco = res["nn_recomputed"]
    obs = res["obs"]
    t = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000.0

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(title, fontsize=11)
    gs = fig.add_gridspec(4, 4, hspace=0.45, wspace=0.35)

    # 1. Trajectory with gates overlaid (in sim NED coordinates)
    ax = fig.add_subplot(gs[0:2, 0:2])
    sim_x =  df["stateEstimate.x"].to_numpy()
    sim_y = -df["stateEstimate.y"].to_numpy()
    ax.plot(sim_x, sim_y, "b-", lw=0.6, label="trajectory (sim NED)")
    if rl_start is not None and rl_start < len(df):
        ax.plot(sim_x[rl_start:], sim_y[rl_start:], "r-", lw=0.8, label="RL phase")
        ax.scatter(sim_x[rl_start], sim_y[rl_start], c="r", s=80, marker="*", zorder=5,
                   label="RL start")
    ax.scatter(sim_x[0], sim_y[0], c="g", s=60, marker="o", label="log start")
    # Gates
    for i in range(NUM_GATES):
        ax.scatter(gx[i], gy[i], s=150, marker="s",
                   edgecolors="k", facecolors="yellow", zorder=4)
        ax.text(gx[i]+0.1, gy[i]+0.1, f"G{i}", fontsize=9)
        # Gate normal arrow (length 0.4)
        ax.arrow(gx[i], gy[i], 0.4*np.cos(gyaw[i]), 0.4*np.sin(gyaw[i]),
                 head_width=0.1, fc="k", ec="k", length_includes_head=True)
    ax.set_xlabel("sim x [m] (NED)")
    ax.set_ylabel("sim y [m] (NED, = -fw_y)")
    ax.set_title("Top-down trajectory + gates (black arrow = gate normal)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    # 2. Target gate index over time
    ax = fig.add_subplot(gs[0, 2])
    ax.step(t, res["tgt_gate"], where="post")
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls="--", lw=0.8)
    ax.set_title("Target gate index")
    ax.set_xlabel("t [s]"); ax.set_ylabel("gate")
    ax.grid(alpha=0.3)

    # 3. Recomputed vs logged NN output
    for k in range(4):
        ax = fig.add_subplot(gs[1 + k//2, 2 + (k%2)])
        ax.plot(t, nn_logged[:, k], "b-", lw=0.6, label="logged")
        ax.plot(t, nn_reco[:, k], "r--", lw=0.5, label="recomputed")
        if rl_start is not None:
            ax.axvline(t.iloc[rl_start], color="k", ls=":", lw=0.8)
        ax.set_title(f"nnOut{k} logged vs recomputed")
        ax.set_xlabel("t [s]")
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
        ax.set_ylim([-1.2, 1.2])

    # 4. Gate-relative position (obs[0-2])
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(t, obs[:, 0], label="Δx (fwd from gate)")
    ax.plot(t, obs[:, 1], label="Δy")
    ax.plot(t, obs[:, 2], label="Δz (down pos=below)")
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("obs[0-2]: gate − drone (NED)")
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)

    # 5. Body velocity (obs[3-5])
    ax = fig.add_subplot(gs[2, 1])
    ax.plot(t, obs[:, 3], label="u (fwd)")
    ax.plot(t, obs[:, 4], label="v (right)")
    ax.plot(t, obs[:, 5], label="w (down)")
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("obs[3-5]: body velocity (sim)")
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)

    # 6. Quaternion (obs[6-9])
    ax = fig.add_subplot(gs[3, 0])
    for k, lbl in enumerate(["qw", "qx", "qy", "qz"]):
        ax.plot(t, obs[:, 6+k], lw=0.6, label=lbl)
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("obs[6-9]: gate-relative quaternion")
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=2)

    # 7. Angular rates (obs[10-12])
    ax = fig.add_subplot(gs[3, 1])
    ax.plot(t, obs[:, 10], label="p (roll)")
    ax.plot(t, obs[:, 11], label="q (pitch,sim)")
    ax.plot(t, obs[:, 12], label="r (yaw,sim)")
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("obs[10-12]: body angular rates")
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)

    # 8. Next gate position + yaw (obs[17-20])
    ax = fig.add_subplot(gs[2, 2])
    ax.plot(t, obs[:, 17], label="Δx_next")
    ax.plot(t, obs[:, 18], label="Δy_next")
    ax.plot(t, obs[:, 19], label="Δz_next")
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("obs[17-19]: next gate − drone")
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)

    ax = fig.add_subplot(gs[3, 2])
    ax.plot(t, obs[:, 20])
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("obs[20]: next_gate_yaw − drone_yaw (wrapped)")
    ax.set_xlabel("t [s]"); ax.set_ylabel("rad"); ax.grid(alpha=0.3)
    ax.axhline(0, color="k", lw=0.4)

    # 9. Residual: logged - recomputed
    ax = fig.add_subplot(gs[2, 3])
    res_diff = res["diff"]
    for k in range(4):
        ax.plot(t, res_diff[:, k], lw=0.5, label=f"Δout{k}")
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("logged − recomputed NN output")
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)

    # 10. Yaw + Δyaw_next — to sanity-check sign
    ax = fig.add_subplot(gs[3, 3])
    ax.plot(t, df["stateEstimate.yaw"], label="fw yaw [deg]", lw=0.6)
    ax.plot(t, -df["stateEstimate.yaw"], label="sim yaw = -fw yaw [deg]", lw=0.4, alpha=0.6)
    if rl_start is not None:
        ax.axvline(t.iloc[rl_start], color="r", ls=":", lw=0.8)
    ax.set_title("Yaw (firmware vs sim convention)")
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)

    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--neural-net-c",
                    default=os.path.join(os.path.dirname(__file__),
                                         "..", "..",
                                         "examples", "app_rl_controller",
                                         "src", "neural_net.c"))
    ap.add_argument("--origin-x", type=float, default=DEFAULT_GATE_ORIGIN[0])
    ap.add_argument("--origin-y", type=float, default=DEFAULT_GATE_ORIGIN[1])
    ap.add_argument("--gate-alt", type=float, default=DEFAULT_GATE_ALT)
    ap.add_argument("--start-row", type=int, default=None,
                    help="Trim log to [start_row:end_row] before analysis")
    ap.add_argument("--end-row", type=int, default=None)
    ap.add_argument("--rl-start", type=int, default=None,
                    help="Override auto-detected RL-start row")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    # Load network
    c_path = os.path.abspath(args.neural_net_c)
    if not os.path.exists(c_path):
        print(f"neural_net.c not found at {c_path}", file=sys.stderr)
        sys.exit(1)
    print(f"Loading network from {c_path}")
    net = load_network(c_path)
    print(f"  layer0: {net[0].shape}   layer1: {net[2].shape}   layer2: {net[4].shape}")

    # Load log
    print(f"Loading log {args.logfile}")
    df = pd.read_csv(args.logfile)
    if args.start_row is not None or args.end_row is not None:
        df = df.iloc[args.start_row:args.end_row].reset_index(drop=True)
    print(f"  {len(df)} rows")

    # Detect RL start
    rl_start = args.rl_start if args.rl_start is not None else detect_rl_start(df)
    if rl_start is not None:
        t_start = (df["timestamp"].iloc[rl_start] - df["timestamp"].iloc[0]) / 1000.0
        print(f"  RL start detected at row {rl_start} (t={t_start:.2f}s)")
    else:
        print("  RL start not detected (no motor saturation observed)")

    # Analyse
    res = analyse(df, net,
                  origin=(args.origin_x, args.origin_y),
                  altitude=args.gate_alt,
                  rl_start=rl_start)

    # Report
    report_anomalies(df, res, rl_start)

    # Plot
    if not args.no_plot:
        plot_diagnostics(df, res, rl_start,
                         title=f"{os.path.basename(args.logfile)}  "
                               f"origin=({args.origin_x},{args.origin_y}) "
                               f"alt={args.gate_alt}")


if __name__ == "__main__":
    main()
