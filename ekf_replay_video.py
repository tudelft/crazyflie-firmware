"""
Matplotlib video comparing two EKF replays against mocap ground truth.

Runs `ekf_replay.run_ekf` twice on the same CSV:
  * baseline — firmware defaults (no model changes)
  * tuned    — the parameters currently active in `ekf_replay.main()`
               (drag, COP, flowdeck lever-arm, retuned noise terms)

Layout (dark theme, slowly rotating 3D view):
  +----------------+-----------------------+
  | vx body (m/s)  |                       |
  +----------------+   3D scene with the   |
  | vy body (m/s)  |   actual drone, mocap |
  +----------------+   trail (grey) and    |
  | vz body (m/s)  |   the two EKF ghosts  |
  +----------------+-----------------------+

Each velocity panel shows three traces:
    mocap  (light grey)  — ground truth
    baseline EKF (orange)
    tuned EKF (blue)

Frames are rendered in parallel with `multiprocessing.Pool` and then
stitched with ffmpeg.

Usage:
  python ekf_replay_video.py
  python ekf_replay_video.py --csv DataFlapperEKF/square_100cm_mtf.csv \
      --out ekf_compare_square_100cm.mp4 --fps 20 --speed 1.0 --workers 10
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.colors as mcolors  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: E402

from ekf_replay import EKFParams, run_ekf  # noqa: E402


# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------

C_MOCAP    = "#ffffff"   # pure white — maximum contrast vs the tuned-EKF blue
C_BASELINE = "#ff8c32"   # orange = EKF without model changes
C_TUNED    = "#1e90ff"   # saturated dodger-blue = EKF with model changes
C_DRONE    = "#5fff7b"   # vibrant green for the actual drone marker
C_PANE     = (0.25, 0.25, 0.35, 0.6)
C_GRID3D   = (0.25, 0.25, 0.35, 0.25)
C_FLOOR    = "#404060"


# ---------------------------------------------------------------------------
# EKF parameter sets
# ---------------------------------------------------------------------------

def make_param_sets() -> tuple[EKFParams, EKFParams]:
    """Return (baseline_params, tuned_params)."""
    baseline = EKFParams()

    tuned = EKFParams(
        proc_noise_acc_xy=1.05006,
        proc_noise_acc_z=0.604273,
        meas_noise_gyro_rp=0.0521776,
        meas_noise_gyro_yaw=0.116742,
        drag_x=4.39468,
        drag_y=2.88896,
        drag_z=0.0611769,
        flow_std_fixed_x=1.07615,
        flow_std_fixed_y=5.41112,
        flow_resolution=0.22987,
        cop=np.array([0.0, 0.0, 0.03]),
        flowdeck_pos=np.array([0.0, 0.0, -0.12]),
    )
    return baseline, tuned


# ---------------------------------------------------------------------------
# Per-worker figure setup
# ---------------------------------------------------------------------------

# Module-level state populated by `_init_worker` in each Pool process.
_W: dict = {}


def _build_figure(payload: dict):
    """Construct the full figure once per worker; returns dict of artists."""
    t            = payload["t"]
    mocap_xyz    = payload["mocap_xyz"]
    baseline_xyz = payload["baseline_xyz"]
    tuned_xyz    = payload["tuned_xyz"]
    mocap_v      = payload["mocap_v"]
    baseline_v   = payload["baseline_v"]
    tuned_v      = payload["tuned_v"]
    v_lo, v_hi   = payload["v_range"]
    xy_lim       = payload["xy_lim"]
    z_lim        = payload["z_lim"]

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(18, 9), facecolor="black")
    gs = fig.add_gridspec(
        3, 5, hspace=0.42, wspace=0.45,
        left=0.06, right=0.98, top=0.90, bottom=0.07,
    )

    ax_vx = fig.add_subplot(gs[0, 0:2], facecolor="black")
    ax_vy = fig.add_subplot(gs[1, 0:2], facecolor="black")
    ax_vz = fig.add_subplot(gs[2, 0:2], facecolor="black")
    side_axes = [ax_vx, ax_vy, ax_vz]

    ax3d = fig.add_subplot(gs[:, 2:5], projection="3d", facecolor="black")
    ax3d.set_xlim(*xy_lim)
    ax3d.set_ylim(*xy_lim)
    ax3d.set_zlim(*z_lim)
    ax3d.set_box_aspect(
        (1, 1, (z_lim[1] - z_lim[0]) / (xy_lim[1] - xy_lim[0]))
    )
    ax3d.set_xlabel("x [m]", color="#aaaaaa", labelpad=-2)
    ax3d.set_ylabel("y [m]", color="#aaaaaa", labelpad=-2)
    ax3d.set_zlabel("z [m]", color="#aaaaaa", labelpad=-2)
    ax3d.tick_params(colors="#888888", labelsize=7)
    for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
        axis.pane.fill = False
        axis.pane.set_edgecolor(C_PANE)
        axis._axinfo["grid"]["color"] = C_GRID3D

    fx = np.array([xy_lim[0], xy_lim[1], xy_lim[1], xy_lim[0], xy_lim[0]])
    fy = np.array([xy_lim[0], xy_lim[0], xy_lim[1], xy_lim[1], xy_lim[0]])
    ax3d.plot(fx, fy, np.zeros_like(fx), color=C_FLOOR, lw=0.8, alpha=0.5)

    # 3D scene shows ONLY the mocap (truth) drone and its trail.
    _dummy = np.zeros((1, 2, 3))
    trail_mocap = Line3DCollection(_dummy, linewidths=2.2)
    ax3d.add_collection3d(trail_mocap)

    glow_specs = [(550, 0.10), (250, 0.22), (110, 0.45), (38, 1.0)]
    drone_glow = []
    for size, alpha in glow_specs:
        s = ax3d.scatter([0], [0], [0], s=size, color=C_DRONE,
                         alpha=alpha, edgecolors="none", depthshade=False)
        drone_glow.append(s)

    stem_mocap, = ax3d.plot([0, 0], [0, 0], [0, 0], color=C_DRONE,
                            alpha=0.25, lw=0.9)

    handles3d = [
        plt.Line2D([0], [0], marker="o", mfc=C_DRONE, mec="none", ls="",
                   ms=9, label="drone (mocap)"),
        plt.Line2D([0], [0], color=C_MOCAP, lw=2.2, label="mocap trail"),
    ]
    ax3d.legend(handles=handles3d, loc="upper left", fontsize=8,
                framealpha=0.0, labelcolor="#dddddd")

    labels = ["vx body  [m/s]", "vy body  [m/s]", "vz body  [m/s]"]
    line_mocap = []; line_base = []; line_tuned = []
    cursor_v   = []; dot_mocap = []; dot_base = []; dot_tuned = []

    for ax, lbl in zip(side_axes, labels):
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(v_lo, v_hi)
        ax.set_ylabel(lbl, color="#dddddd", fontsize=10)
        ax.tick_params(colors="#888888", labelsize=8)
        for sp in ax.spines.values():
            sp.set_color("#333333")
        ax.grid(True, alpha=0.12)
        ax.axhline(0, color="#444444", lw=0.6)

        lm, = ax.plot([], [], color=C_MOCAP,    lw=2.0, label="mocap")
        lb, = ax.plot([], [], color=C_BASELINE, lw=1.4, label="EKF baseline",
                      alpha=0.95)
        lt, = ax.plot([], [], color=C_TUNED,    lw=1.4, label="EKF tuned",
                      alpha=0.95)
        line_mocap.append(lm); line_base.append(lb); line_tuned.append(lt)

        cursor_v.append(ax.axvline(t[0], color="white", lw=0.7, alpha=0.3))
        dm, = ax.plot([], [], "o", color=C_MOCAP,    ms=5, mec="black", mew=0.4)
        db, = ax.plot([], [], "o", color=C_BASELINE, ms=4, mec="black", mew=0.4)
        dt_, = ax.plot([], [], "o", color=C_TUNED,   ms=4, mec="black", mew=0.4)
        dot_mocap.append(dm); dot_base.append(db); dot_tuned.append(dt_)

    side_axes[0].legend(loc="upper right", fontsize=8, framealpha=0.0,
                        labelcolor="#dddddd", ncol=3)
    side_axes[0].set_title("Body-frame velocity",
                           color="#dddddd", fontsize=11, pad=8)
    side_axes[-1].set_xlabel("time [s]", color="#aaaaaa", fontsize=10)

    title    = fig.suptitle("", color="white", fontsize=13, fontweight="bold")
    fig.text(
        0.06, 0.945,
        "Mocap (white) vs EKF baseline (orange) vs EKF tuned (blue)",
        color="#9a9aff", fontsize=10,
    )

    return {
        "fig": fig, "ax3d": ax3d,
        "trail_mocap": trail_mocap,
        "drone_glow": drone_glow, "stem_mocap": stem_mocap,
        "line_mocap": line_mocap, "line_base": line_base, "line_tuned": line_tuned,
        "cursor_v": cursor_v, "dot_mocap": dot_mocap,
        "dot_base": dot_base, "dot_tuned": dot_tuned,
        "title": title,
    }


def _fade_segments(path: np.ndarray, color):
    segs = np.stack([path[:-1], path[1:]], axis=1)
    rgb = mcolors.to_rgb(color)
    alphas = np.linspace(0.05, 0.95, len(segs))
    rgba = np.array([[*rgb, a] for a in alphas])
    return segs, rgba


def _update_frame(state, payload, fi: int) -> None:
    """Mutate the cached artists to reflect frame `fi`."""
    t            = payload["t"]
    mocap_xyz    = payload["mocap_xyz"]
    baseline_xyz = payload["baseline_xyz"]
    tuned_xyz    = payload["tuned_xyz"]
    mocap_v      = payload["mocap_v"]
    baseline_v   = payload["baseline_v"]
    tuned_v      = payload["tuned_v"]
    frame_idx    = payload["frame_idx"]
    trail_samples = payload["trail_samples"]
    duration_s    = payload["duration_s"]
    init_azim     = payload["init_azim"]
    init_elev     = payload["init_elev"]

    idx = int(frame_idx[fi])
    trail_start = max(0, idx - trail_samples)
    t_now = float(t[idx])

    # 3D scene: only the mocap trail.
    path = mocap_xyz[trail_start:idx + 1]
    if len(path) >= 2:
        segs, rgba = _fade_segments(path, C_MOCAP)
        state["trail_mocap"].set_segments(segs)
        state["trail_mocap"].set_colors(rgba)
    else:
        state["trail_mocap"].set_segments([])

    xm, ym, zm = mocap_xyz[idx]
    for s in state["drone_glow"]:
        s._offsets3d = ([xm], [ym], [zm])
    state["stem_mocap"].set_data_3d([xm, xm], [ym, ym], [0.0, zm])

    xs = t[:idx + 1]
    for i in range(3):
        state["line_mocap"][i].set_data(xs, mocap_v[:idx + 1, i])
        state["line_base"][i].set_data(xs,  baseline_v[:idx + 1, i])
        state["line_tuned"][i].set_data(xs, tuned_v[:idx + 1, i])
        state["cursor_v"][i].set_xdata([t_now, t_now])
        state["dot_mocap"][i].set_data([t_now], [mocap_v[idx, i]])
        state["dot_base"][i].set_data([t_now],  [baseline_v[idx, i]])
        state["dot_tuned"][i].set_data([t_now], [tuned_v[idx, i]])

    elapsed = t_now - t[0]
    state["title"].set_text(
        f"t = {elapsed:6.2f} s / {duration_s:5.1f} s"
    )
    state["ax3d"].view_init(elev=init_elev, azim=init_azim + fi * 0.12)


# ---------------------------------------------------------------------------
# Multiprocessing worker
# ---------------------------------------------------------------------------

def _init_worker(payload: dict, out_dir: str, dpi: int):
    _W["payload"] = payload
    _W["state"]   = _build_figure(payload)
    _W["out_dir"] = Path(out_dir)
    _W["dpi"]     = dpi


def _render_one(fi: int) -> int:
    state = _W["state"]
    _update_frame(state, _W["payload"], fi)
    out = _W["out_dir"] / f"frame_{fi:06d}.png"
    state["fig"].savefig(out, dpi=_W["dpi"], facecolor="black")
    return fi


# ---------------------------------------------------------------------------
# Main driver
# ---------------------------------------------------------------------------

def render_video(csv_path: Path, out_path: Path, fps: int, speed: float,
                 imu_rate: float, trail_s: float | None, workers: int,
                 dpi: int) -> None:
    baseline_params, tuned_params = make_param_sets()

    print(f"[1/4] Running baseline EKF on {csv_path.name} ...", flush=True)
    base = run_ekf(csv_path, imu_rate=imu_rate, params=baseline_params)
    print(f"[2/4] Running tuned EKF on {csv_path.name} ...", flush=True)
    tuned = run_ekf(csv_path, imu_rate=imu_rate, params=tuned_params)

    n = min(len(base), len(tuned))
    base = base.iloc[:n].reset_index(drop=True)
    tuned = tuned.iloc[:n].reset_index(drop=True)
    t = base["time"].to_numpy()

    mocap_xyz    = np.column_stack([base["ls_x"], base["ls_y"], base["ls_z"]])
    baseline_xyz = np.column_stack([base["x"],    base["y"],    base["z"]])
    tuned_xyz    = np.column_stack([tuned["x"],   tuned["y"],   tuned["z"]])

    mocap_v    = np.column_stack([base["ls_vx_b"], base["ls_vy_b"], base["ls_vz_b"]])
    baseline_v = np.column_stack([base["vx_b"],    base["vy_b"],    base["vz_b"]])
    tuned_v    = np.column_stack([tuned["vx_b"],   tuned["vy_b"],   tuned["vz_b"]])

    all_v = np.concatenate([mocap_v.ravel(), baseline_v.ravel(), tuned_v.ravel()])
    all_v = all_v[np.isfinite(all_v)]
    v_lo, v_hi = np.percentile(all_v, [0.5, 99.5])
    pad = max((v_hi - v_lo) * 0.12, 0.05)
    v_lo -= pad; v_hi += pad

    x_lo, x_hi = float(np.nanmin(mocap_xyz[:, 0])), float(np.nanmax(mocap_xyz[:, 0]))
    y_lo, y_hi = float(np.nanmin(mocap_xyz[:, 1])), float(np.nanmax(mocap_xyz[:, 1]))
    z_max      = float(np.nanmax(mocap_xyz[:, 2]))
    half = max(abs(x_lo), abs(x_hi), abs(y_lo), abs(y_hi)) + 0.25
    xy_lim = (-half, half)
    z_lim  = (0.0, max(z_max + 0.25, 1.5))

    src_rate = 1.0 / float(np.mean(np.diff(t)))
    stride = max(1, int(round(src_rate / (fps / max(speed, 1e-6)))))
    frame_idx = np.arange(0, n, stride)
    n_frames = len(frame_idx)
    duration_s = float(t[-1] - t[0])
    trail_samples = (n + 1) if trail_s is None else int(round(trail_s * src_rate))

    print(f"[3/4] {n_frames} frames @ {fps} fps  (flight {duration_s:.1f} s, "
          f"speed {speed}x, body-v y-range [{v_lo:+.2f}, {v_hi:+.2f}])",
          flush=True)

    payload = {
        "t": t,
        "mocap_xyz": mocap_xyz, "baseline_xyz": baseline_xyz, "tuned_xyz": tuned_xyz,
        "mocap_v":   mocap_v,   "baseline_v":   baseline_v,   "tuned_v":   tuned_v,
        "v_range": (v_lo, v_hi),
        "xy_lim": xy_lim, "z_lim": z_lim,
        "frame_idx": frame_idx, "trail_samples": trail_samples,
        "duration_s": duration_s,
        "init_azim": -55.0, "init_elev": 22.0,
    }

    # Parallel frame rendering
    workers = max(1, workers)
    tmp_dir = Path(tempfile.mkdtemp(prefix="ekf_frames_"))
    print(f"[4/4] Rendering with {workers} workers → {tmp_dir}", flush=True)
    t0 = time.time()
    try:
        ctx = mp.get_context("fork")
        with ctx.Pool(processes=workers,
                      initializer=_init_worker,
                      initargs=(payload, str(tmp_dir), dpi)) as pool:
            done = 0
            for fi in pool.imap_unordered(_render_one, range(n_frames),
                                          chunksize=4):
                done += 1
                if done % 40 == 0 or done == n_frames:
                    rate = done / (time.time() - t0)
                    eta  = (n_frames - done) / max(rate, 1e-9)
                    print(f"  {done:4d}/{n_frames}  "
                          f"({rate:4.1f} f/s, ETA {eta:5.1f}s)", flush=True)

        elapsed = time.time() - t0
        print(f"  rendered {n_frames} frames in {elapsed:.1f}s "
              f"({n_frames / elapsed:.1f} f/s)", flush=True)

        print("Encoding with ffmpeg ...", flush=True)
        ff_cmd = [
            "ffmpeg", "-y", "-loglevel", "warning",
            "-framerate", str(fps),
            "-i", str(tmp_dir / "frame_%06d.png"),
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-crf", "20",
            "-preset", "medium",
            "-movflags", "+faststart",
            str(out_path),
        ]
        subprocess.run(ff_cmd, check=True)
        print(f"Saved {out_path}", flush=True)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--csv", type=Path,
                        default=Path("DataFlapperEKF/square_100cm_mtf.csv"))
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--imu-rate", type=float, default=500.0)
    parser.add_argument("--trail", type=float, default=None,
                        help="3D trail length in seconds (default: full flight)")
    parser.add_argument("--workers", type=int, default=10,
                        help="Parallel rendering workers (default: 10)")
    parser.add_argument("--dpi", type=int, default=100)
    args = parser.parse_args()

    if not args.csv.exists():
        print(f"CSV not found: {args.csv}", file=sys.stderr)
        sys.exit(1)
    out_path = args.out or Path(f"ekf_compare_{args.csv.stem}.mp4")

    render_video(args.csv, out_path, args.fps, args.speed,
                 args.imu_rate, args.trail, args.workers, args.dpi)


if __name__ == "__main__":
    main()
