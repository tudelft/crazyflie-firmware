"""
mocap_utils — Unified analysis and plotting for mocap + flapper flight data.

Classes
-------
MocapProcessor
    Load a mocap CSV (Z-fwd / X-left / Y-up), transform to FLU
    (X-fwd / Y-left / Z-up), extract Euler angles, compute smoothed
    world- and body-frame velocities & angular rates.

FlapperProcessor
    Load a Crazyflie/Flapper SD-card CSV, convert firmware units
    (mm/s → m/s, mrad/s → deg/s).

FlightComparison
    Time-sync mocap and flapper data via cross-correlation on horizontal
    speed magnitude.  Estimate the yaw offset between the flapper's
    internal global frame and the mocap world frame.  Rotate flapper
    velocity into the mocap global frame (yaw₀) and then into the body
    frame (using per-sample mocap attitude).  Provides overlay plots of
    mocap vs flapper body-frame velocity.

Quick-start
-----------
    from mocap_utils import MocapProcessor, FlapperProcessor, FlightComparison

    mp = MocapProcessor("mocap.csv")
    mp.compute_velocity()
    mp.compute_rates()

    fp = FlapperProcessor("flapper.csv")

    fc = FlightComparison(mp, fp)
    fc.plot_body_velocity_comparison()
    fc.plot_world_velocity_comparison()
    fc.plot_yaw_rate_comparison()
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import interp1d
from scipy.signal import correlate

# ====================================================================== #
# Constants
# ====================================================================== #

# Change-of-basis matrix: mocap (X-left, Y-up, Z-fwd) → FLU (X-fwd, Y-left, Z-up)
_T = np.array([
    [0, 0, 1],   # X_new(fwd)  = Z_old
    [1, 0, 0],   # Y_new(left) = X_old
    [0, 1, 0],   # Z_new(up)   = Y_old
])


# ====================================================================== #
# MocapProcessor
# ====================================================================== #

class MocapProcessor:
    """Load a mocap CSV, transform to FLU, and compute derived quantities.

    Expected CSV columns (defaults):
        Time (Seconds), pX, pY, pZ, qX, qY, qZ, qW

    After construction the ``.df`` DataFrame contains at minimum:
        Time (Seconds), X, Y, Z, roll, pitch, yaw

    Call :meth:`compute_velocity` to add world- and body-frame velocities.
    Call :meth:`compute_rates` to add smoothed angular rates.
    """

    # --------------------------------------------------------------------- #
    # Construction
    # --------------------------------------------------------------------- #
    def __init__(
        self,
        csv_path: str,
        *,
        time_col: str = "Time (Seconds)",
        pos_cols: tuple[str, str, str] = ("pX", "pY", "pZ"),
        quat_cols: tuple[str, str, str, str] = ("qX", "qY", "qZ", "qW"),
        interpolate: bool = True,
    ) -> None:
        self._time_col = time_col
        self.df = pd.read_csv(csv_path)

        if interpolate:
            self.df.interpolate(method="linear", inplace=True)

        self._transform_position(pos_cols)
        self._transform_orientation(quat_cols)

    # --------------------------------------------------------------------- #
    # Core transforms (called from __init__)
    # --------------------------------------------------------------------- #
    def _transform_position(self, cols: tuple[str, str, str]) -> None:
        pos_old = self.df[list(cols)].values
        pos_new = (_T @ pos_old.T).T
        self.df["X"] = pos_new[:, 0]
        self.df["Y"] = pos_new[:, 1]
        self.df["Z"] = pos_new[:, 2]

    def _transform_orientation(self, cols: tuple[str, str, str, str]) -> None:
        quats = self.df[list(cols)].values          # scipy [x, y, z, w]
        rot_old = R.from_quat(quats)
        R_old = rot_old.as_matrix()                 # (N, 3, 3)
        R_new = np.einsum("ia,nab,jb->nij", _T, R_old, _T)  # T @ R_old @ T^T
        self._rot = R.from_matrix(R_new)

        euler = self._rot.as_euler("ZYX", degrees=True)
        self.df["yaw"]   = euler[:, 0]
        self.df["pitch"] = euler[:, 1]
        self.df["roll"]  = euler[:, 2]

    # --------------------------------------------------------------------- #
    # Derived quantities
    # --------------------------------------------------------------------- #
    @property
    def t(self) -> np.ndarray:
        """Time vector."""
        return self.df[self._time_col].values

    @property
    def dt(self) -> float:
        """Mean sample interval."""
        return float(np.mean(np.diff(self.t)))

    @property
    def rotations(self) -> R:
        """Scipy Rotation array (body→world in FLU)."""
        return self._rot

    def compute_velocity(
        self,
        filter_order: int = 3,
        cutoff_freq: float = 0.05,
    ) -> pd.DataFrame:
        """Compute smoothed world-frame and body-frame velocity.

        Adds columns: vx_world, vy_world, vz_world,
                       vx_body,  vy_body,  vz_body

        Returns ``self.df`` for chaining.
        """
        import pynumdiff

        dt = self.dt
        params = [filter_order, cutoff_freq]

        _, vx = pynumdiff.smooth_finite_difference.butterdiff(
            self.df["X"].values, dt, params)
        _, vy = pynumdiff.smooth_finite_difference.butterdiff(
            self.df["Y"].values, dt, params)
        _, vz = pynumdiff.smooth_finite_difference.butterdiff(
            self.df["Z"].values, dt, params)

        vel_world = np.column_stack([vx, vy, vz])
        vel_body = self._rot.inv().apply(vel_world)

        self.df["vx_world"] = vx
        self.df["vy_world"] = vy
        self.df["vz_world"] = vz
        self.df["vx_body"] = vel_body[:, 0]
        self.df["vy_body"] = vel_body[:, 1]
        self.df["vz_body"] = vel_body[:, 2]

        return self.df

    def compute_rates(
        self,
        filter_order: int = 3,
        cutoff_freq: float = 0.05,
    ) -> pd.DataFrame:
        """Compute smoothed angular rates from Euler angles.

        Adds columns: rollrate_dps, pitchrate_dps, yawrate_dps

        Returns ``self.df`` for chaining.
        """
        import pynumdiff

        dt = self.dt
        params = [filter_order, cutoff_freq]

        _, roll_rate = pynumdiff.smooth_finite_difference.butterdiff(
            self.df["roll"].values, dt, params)
        _, pitch_rate = pynumdiff.smooth_finite_difference.butterdiff(
            self.df["pitch"].values, dt, params)
        _, yaw_rate = pynumdiff.smooth_finite_difference.butterdiff(
            self.df["yaw"].values, dt, params)

        self.df["rollrate_dps"]  = roll_rate
        self.df["pitchrate_dps"] = pitch_rate
        self.df["yawrate_dps"]   = yaw_rate

        return self.df

    # --------------------------------------------------------------------- #
    # Plotting helpers
    # --------------------------------------------------------------------- #
    def plot_trajectory_3d(self, figsize=(10, 8)):
        """3D scatter coloured by time."""
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, projection="3d")
        sc = ax.scatter(self.df["X"], self.df["Y"], self.df["Z"],
                        c=self.t, cmap="viridis", s=2)
        ax.scatter(*self.df[["X", "Y", "Z"]].iloc[0],
                   color="red", s=100, marker="o", label="Start")
        ax.set_xlabel("X (forward)")
        ax.set_ylabel("Y (left)")
        ax.set_zlabel("Z (up)")
        ax.set_title("Transformed 3D Trajectory")
        plt.colorbar(sc, label="Time (s)")
        ax.legend()
        plt.tight_layout()
        return fig, ax

    def plot_position_vs_time(self, figsize=(12, 8)):
        """X, Y, Z position vs time."""
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)

        for ax, col, label, colour in [
            (axes[0], "X", "X (forward)", "tab:blue"),
            (axes[1], "Y", "Y (left)",    "tab:orange"),
            (axes[2], "Z", "Z (up)",      "tab:green"),
        ]:
            ax.plot(self.t, self.df[col], label=label, color=colour)
            ax.set_ylabel(label)
            ax.legend()
            ax.grid(True)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle("Position vs Time (FLU)", fontsize=14)
        plt.tight_layout()
        return fig, axes

    def plot_euler_vs_time(self, figsize=(12, 8)):
        """Roll, pitch, yaw vs time."""
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)

        for ax, col, colour in [
            (axes[0], "roll",  "tab:blue"),
            (axes[1], "pitch", "tab:orange"),
            (axes[2], "yaw",   "tab:green"),
        ]:
            ax.plot(self.t, self.df[col], label=col.capitalize(), color=colour)
            ax.set_ylabel(f"{col.capitalize()} (deg)")
            ax.legend()
            ax.grid(True)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle("Euler Angles vs Time (FLU)", fontsize=14)
        plt.tight_layout()
        return fig, axes

    def plot_topdown_heading(self, n_arrows: int = 40, arrow_len: float = 0.05,
                             figsize=(10, 8)):
        """Top-down XY plot with heading arrows."""
        fig, ax = plt.subplots(figsize=figsize)
        ax.plot(self.df["X"], self.df["Y"], "k-", alpha=0.3, linewidth=0.5)
        sc = ax.scatter(self.df["X"], self.df["Y"],
                        c=self.t, cmap="viridis", s=3)

        step = max(1, len(self.df) // n_arrows)
        for i in range(0, len(self.df), step):
            yaw_rad = np.radians(self.df["yaw"].iloc[i])
            dx = arrow_len * np.cos(yaw_rad)
            dy = arrow_len * np.sin(yaw_rad)
            ax.annotate(
                "", xy=(self.df["X"].iloc[i] + dx, self.df["Y"].iloc[i] + dy),
                xytext=(self.df["X"].iloc[i], self.df["Y"].iloc[i]),
                arrowprops=dict(arrowstyle="->", color="red", lw=1.5),
            )

        ax.scatter(self.df["X"].iloc[0], self.df["Y"].iloc[0],
                   color="red", s=100, zorder=5, label="Start")
        ax.set_xlabel("X (forward)")
        ax.set_ylabel("Y (left)")
        ax.set_title("Top-down XY with Heading Arrows")
        ax.set_aspect("equal")
        ax.legend()
        ax.grid(True)
        plt.colorbar(sc, label="Time (s)")
        plt.tight_layout()
        return fig, ax

    def plot_world_velocity(self, figsize=(12, 8)):
        """World-frame velocity (requires :meth:`compute_velocity` first)."""
        self._require_cols("vx_world", "vy_world", "vz_world",
                           method="compute_velocity")
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)

        for ax, col, label, colour in [
            (axes[0], "vx_world", "Vx world (forward)", "tab:blue"),
            (axes[1], "vy_world", "Vy world (left)",    "tab:orange"),
            (axes[2], "vz_world", "Vz world (up)",      "tab:green"),
        ]:
            ax.plot(self.t, self.df[col], label=label, color=colour)
            ax.set_ylabel(col.split("_")[0].capitalize() + " (m/s)")
            ax.legend()
            ax.grid(True)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle("World-Frame Velocity (smoothed)", fontsize=14)
        plt.tight_layout()
        return fig, axes

    def plot_body_velocity(self, figsize=(12, 8)):
        """Body-frame velocity (requires :meth:`compute_velocity` first)."""
        self._require_cols("vx_body", "vy_body", "vz_body",
                           method="compute_velocity")
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)

        for ax, col, label, colour in [
            (axes[0], "vx_body", "Vx body (forward)", "tab:blue"),
            (axes[1], "vy_body", "Vy body (left)",    "tab:orange"),
            (axes[2], "vz_body", "Vz body (up)",      "tab:green"),
        ]:
            ax.plot(self.t, self.df[col], label=label, color=colour)
            ax.set_ylabel(col.split("_")[0].capitalize() + " (m/s)")
            ax.legend()
            ax.grid(True)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle("Body-Frame Velocity (smoothed)", fontsize=14)
        plt.tight_layout()
        return fig, axes

    def plot_rates(self, figsize=(12, 8)):
        """Angular rates (requires :meth:`compute_rates` first)."""
        self._require_cols("rollrate_dps", "pitchrate_dps", "yawrate_dps",
                           method="compute_rates")
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)

        for ax, col, label, colour in [
            (axes[0], "rollrate_dps",  "Roll rate",  "tab:blue"),
            (axes[1], "pitchrate_dps", "Pitch rate", "tab:orange"),
            (axes[2], "yawrate_dps",   "Yaw rate",   "tab:green"),
        ]:
            ax.plot(self.t, self.df[col], label=label, color=colour)
            ax.set_ylabel(f"{label} (deg/s)")
            ax.legend()
            ax.grid(True)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle("Angular Rates (smoothed)", fontsize=14)
        plt.tight_layout()
        return fig, axes

    # --------------------------------------------------------------------- #
    # Internal helpers
    # --------------------------------------------------------------------- #
    def _require_cols(self, *cols: str, method: str) -> None:
        missing = [c for c in cols if c not in self.df.columns]
        if missing:
            raise RuntimeError(
                f"Columns {missing} not found — call .{method}() first."
            )


# ====================================================================== #
# FlapperProcessor
# ====================================================================== #

class FlapperProcessor:
    """Load a Crazyflie / Flapper SD-card CSV and convert units.

    Firmware conventions
    --------------------
    * ``stateEstimateZ.vx / vy / vz`` — velocity in the drone's internal
      global frame, units **mm/s**.
    * ``stateEstimateZ.rateYaw`` — yaw rate in **millirad/s**.

    After construction ``self.df`` contains columns:
        t_s, vx_global, vy_global, vz_global  (m/s, flapper internal global)
        yawrate_dps                            (deg/s)
    """

    def __init__(
        self,
        csv_path: str,
        *,
        time_col: str = "t_s",
        vx_col: str = "stateEstimateZ.vx",
        vy_col: str = "stateEstimateZ.vy",
        vz_col: str = "stateEstimateZ.vz",
        yawrate_col: str = "stateEstimateZ.rateYaw",
        interpolate: bool = True,
    ) -> None:
        self._time_col = time_col
        self.df = pd.read_csv(csv_path)

        if interpolate:
            self.df.interpolate(method="linear", inplace=True)

        # Unit conversions
        self.df["vx_global"] = self.df[vx_col] / 1000.0        # mm/s → m/s
        self.df["vy_global"] = self.df[vy_col] / 1000.0
        self.df["vz_global"] = self.df[vz_col] / 1000.0
        self.df["yawrate_dps"] = (
            self.df[yawrate_col] / 1000.0 * (180.0 / np.pi)    # mrad/s → deg/s
        )

    @property
    def t(self) -> np.ndarray:
        """Time vector (seconds)."""
        return self.df[self._time_col].values

    @property
    def dt(self) -> float:
        return float(np.mean(np.diff(self.t)))

    # ---- standalone plots ------------------------------------------------ #
    def plot_velocity(self, figsize=(12, 10)):
        """Plot flapper global-frame velocity and yaw rate."""
        fig, axes = plt.subplots(4, 1, figsize=figsize, sharex=True)

        for ax, col, label, colour in [
            (axes[0], "vx_global", "Vx (m/s)", "tab:red"),
            (axes[1], "vy_global", "Vy (m/s)", "tab:purple"),
            (axes[2], "vz_global", "Vz (m/s)", "tab:cyan"),
            (axes[3], "yawrate_dps", "Yaw rate (deg/s)", "tab:green"),
        ]:
            ax.plot(self.t, self.df[col], label=label, color=colour)
            ax.set_ylabel(label)
            ax.legend()
            ax.grid(True)

        axes[3].set_xlabel("Time (s)")
        fig.suptitle("Flapper Onboard Data (internal global frame)", fontsize=14)
        plt.tight_layout()
        return fig, axes


# ====================================================================== #
# FlightComparison
# ====================================================================== #

class FlightComparison:
    """Synchronise mocap and flapper data and produce comparison plots.

    On construction this class:
    1. Runs cross-correlation on horizontal speed magnitude to find the
       time offset between the two recordings.
    2. Estimates the yaw offset *yaw₀* — the mocap-world yaw of the
       drone at flapper boot time.
    3. Rotates flapper velocity from the internal global frame into the
       mocap world frame (using *yaw₀*).
    4. Computes flapper velocity in the body frame by applying the
       inverse of the mocap attitude at each time step.

    Parameters
    ----------
    mocap : MocapProcessor
        Must have had :meth:`compute_velocity` and :meth:`compute_rates`
        called already.
    flapper : FlapperProcessor
        Freshly loaded flapper data.
    dt_resample : float
        Common time grid spacing for cross-correlation (default 0.01 s).
    init_window : float
        Window (seconds from flapper boot) over which the average mocap
        yaw is used to estimate *yaw₀* (default 2.0 s).
    """

    def __init__(
        self,
        mocap: MocapProcessor,
        flapper: FlapperProcessor,
        *,
        dt_resample: float = 0.01,
        init_window: float = 2.0,
    ) -> None:
        self.mocap = mocap
        self.flapper = flapper

        mocap._require_cols(
            "vx_world", "vy_world", "vz_world",
            "vx_body", "vy_body", "vz_body",
            method="compute_velocity",
        )

        # ---- step 1: time sync via cross-correlation on |v_horiz| ---- #
        self.time_offset = self._cross_correlate_time(dt_resample)
        self.flapper.df["t_aligned"] = self.flapper.t + self.time_offset

        # ---- step 2: estimate yaw₀ ---- #
        self.yaw_0 = self._estimate_yaw_0(init_window)

        # ---- step 3: rotate flapper velocity → mocap world frame ---- #
        self._rotate_flapper_to_world()

        # ---- step 4: rotate flapper velocity → body frame ---- #
        self._rotate_flapper_to_body()

        print(
            f"FlightComparison ready\n"
            f"  time offset (mocap − flapper) : {self.time_offset:+.3f} s\n"
            f"  yaw₀ (flapper boot → mocap)   : {self.yaw_0:+.1f}°"
        )

    # ------------------------------------------------------------------ #
    # Internal pipeline steps
    # ------------------------------------------------------------------ #
    def _cross_correlate_time(self, dt_resample: float) -> float:
        """Return *mocap_t − flapper_t* offset via horizontal-speed xcorr."""
        m_hs = np.sqrt(
            self.mocap.df["vx_world"].values ** 2
            + self.mocap.df["vy_world"].values ** 2
        )
        f_hs = np.sqrt(
            self.flapper.df["vx_global"].values ** 2
            + self.flapper.df["vy_global"].values ** 2
        )

        t_start = max(self.mocap.t.min(), self.flapper.t.min())
        t_end = min(self.mocap.t.max(), self.flapper.t.max())
        t_common = np.arange(t_start, t_end, dt_resample)

        m_resampled = interp1d(
            self.mocap.t, m_hs, bounds_error=False, fill_value=0
        )(t_common)
        f_resampled = interp1d(
            self.flapper.t, f_hs, bounds_error=False, fill_value=0
        )(t_common)

        corr = correlate(
            m_resampled - m_resampled.mean(),
            f_resampled - f_resampled.mean(),
            mode="full",
        )
        lags = np.arange(-len(t_common) + 1, len(t_common)) * dt_resample
        return float(lags[np.argmax(corr)])

    def _estimate_yaw_0(self, window: float) -> float:
        """Average mocap yaw over the first *window* seconds after flapper boot."""
        boot_mocap_t = self.time_offset  # flapper t=0 → mocap t=offset
        mask = (
            (self.mocap.t >= boot_mocap_t)
            & (self.mocap.t <= boot_mocap_t + window)
        )
        if mask.sum() > 0:
            return float(np.mean(self.mocap.df.loc[mask, "yaw"]))
        # fallback: first 100 mocap samples
        return float(self.mocap.df["yaw"].iloc[:100].mean())

    def _rotate_flapper_to_world(self) -> None:
        """Apply yaw₀ 2-D rotation: flapper global → mocap world."""
        rad = np.radians(self.yaw_0)
        c, s = np.cos(rad), np.sin(rad)
        vx = self.flapper.df["vx_global"].values
        vy = self.flapper.df["vy_global"].values
        self.flapper.df["vx_world"] = c * vx - s * vy
        self.flapper.df["vy_world"] = s * vx + c * vy
        self.flapper.df["vz_world"] = self.flapper.df["vz_global"].values

    def _rotate_flapper_to_body(self) -> None:
        """Rotate flapper world-frame velocity into the body frame.

        Uses Slerp-interpolated mocap attitude evaluated at each aligned
        flapper time stamp, then applies R⁻¹ (world → body).
        """
        t_m = self.mocap.t
        t_f = self.flapper.df["t_aligned"].values

        # Clamp flapper times to the mocap time range for safe interpolation
        t_lo, t_hi = t_m[0], t_m[-1]
        t_f_clamped = np.clip(t_f, t_lo, t_hi)

        # Slerp: interpolate mocap rotations to flapper time stamps
        slerp = Slerp(t_m, self.mocap.rotations)
        rot_at_flap = slerp(t_f_clamped)

        vel_world = np.column_stack([
            self.flapper.df["vx_world"].values,
            self.flapper.df["vy_world"].values,
            self.flapper.df["vz_world"].values,
        ])
        vel_body = rot_at_flap.inv().apply(vel_world)

        self.flapper.df["vx_body"] = vel_body[:, 0]
        self.flapper.df["vy_body"] = vel_body[:, 1]
        self.flapper.df["vz_body"] = vel_body[:, 2]

    # ------------------------------------------------------------------ #
    # Comparison plots
    # ------------------------------------------------------------------ #
    def plot_body_velocity_comparison(self, figsize=(14, 10)):
        """Overlay mocap and flapper body-frame velocity (vx, vy, vz).

        This is the primary calibration plot — both signals should track
        closely if the estimator is accurate.
        """
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
        t_m = self.mocap.t
        t_f = self.flapper.df["t_aligned"].values

        for ax, m_col, f_col, label in [
            (axes[0], "vx_body", "vx_body", "Forward (Vx)"),
            (axes[1], "vy_body", "vy_body", "Left (Vy)"),
            (axes[2], "vz_body", "vz_body", "Up (Vz)"),
        ]:
            ax.plot(t_m, self.mocap.df[m_col], label="Mocap", color="tab:blue",
                    alpha=0.85, linewidth=1.2)
            ax.plot(t_f, self.flapper.df[f_col], label="Flapper", color="tab:red",
                    alpha=0.75, linewidth=1.0)
            ax.set_ylabel(f"{label} (m/s)")
            ax.legend(loc="upper right")
            ax.grid(True, alpha=0.4)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle(
            f"Body-Frame Velocity — Mocap vs Flapper\n"
            f"(Δt={self.time_offset:+.3f} s, yaw₀={self.yaw_0:+.1f}°)",
            fontsize=14,
        )
        plt.tight_layout()
        return fig, axes

    def plot_world_velocity_comparison(self, figsize=(14, 10)):
        """Overlay mocap and flapper world-frame velocity."""
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
        t_m = self.mocap.t
        t_f = self.flapper.df["t_aligned"].values

        for ax, m_col, f_col, label in [
            (axes[0], "vx_world", "vx_world", "X forward (Vx)"),
            (axes[1], "vy_world", "vy_world", "Y left (Vy)"),
            (axes[2], "vz_world", "vz_world", "Z up (Vz)"),
        ]:
            ax.plot(t_m, self.mocap.df[m_col], label="Mocap", color="tab:blue",
                    alpha=0.85, linewidth=1.2)
            ax.plot(t_f, self.flapper.df[f_col], label="Flapper", color="tab:red",
                    alpha=0.75, linewidth=1.0)
            ax.set_ylabel(f"{label} (m/s)")
            ax.legend(loc="upper right")
            ax.grid(True, alpha=0.4)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle(
            f"World-Frame Velocity — Mocap vs Flapper (rotated by yaw₀={self.yaw_0:+.1f}°)",
            fontsize=14,
        )
        plt.tight_layout()
        return fig, axes

    def plot_yaw_rate_comparison(self, figsize=(12, 5)):
        """Overlay mocap and flapper yaw rate."""
        self.mocap._require_cols("yawrate_dps", method="compute_rates")

        fig, ax = plt.subplots(figsize=figsize)
        ax.plot(self.mocap.t, self.mocap.df["yawrate_dps"],
                label="Mocap (d/dt yaw)", color="tab:blue", alpha=0.85)
        ax.plot(self.flapper.df["t_aligned"], self.flapper.df["yawrate_dps"],
                label="Flapper", color="tab:red", alpha=0.75)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Yaw rate (deg/s)")
        ax.set_title("Yaw Rate — Mocap vs Flapper")
        ax.legend()
        ax.grid(True, alpha=0.4)
        plt.tight_layout()
        return fig, ax

    def plot_three_way_velocity(self, figsize=(14, 10)):
        """3-way comparison: mocap world, flapper raw, flapper rotated."""
        fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
        t_m = self.mocap.t
        t_f = self.flapper.df["t_aligned"].values

        for ax, m_col, f_raw, f_rot, axis_lbl in [
            (axes[0], "vx_world", "vx_global", "vx_world", "X (forward)"),
            (axes[1], "vy_world", "vy_global", "vy_world", "Y (left)"),
            (axes[2], "vz_world", "vz_global", "vz_world", "Z (up)"),
        ]:
            ax.plot(t_m, self.mocap.df[m_col],
                    label="Mocap (world)", color="tab:blue", alpha=0.85)
            ax.plot(t_f, self.flapper.df[f_raw],
                    label="Flapper raw (no rotation)", color="tab:orange",
                    alpha=0.7, linestyle="--")
            ax.plot(t_f, self.flapper.df[f_rot],
                    label=f"Flapper rotated (yaw₀={self.yaw_0:+.1f}°)",
                    color="tab:green", alpha=0.8, linestyle="-.")
            ax.set_ylabel(f"V{axis_lbl[0].lower()} (m/s)")
            ax.set_title(f"{axis_lbl} velocity")
            ax.legend(loc="upper right", fontsize=9)
            ax.grid(True, alpha=0.4)

        axes[2].set_xlabel("Time (s)")
        fig.suptitle("3-Way Comparison: Mocap world / Flapper raw / Flapper rotated",
                     fontsize=14)
        plt.tight_layout()
        return fig, axes
