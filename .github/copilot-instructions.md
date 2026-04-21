# Project Guidelines

## Build and Test

See [CLAUDE.md](../CLAUDE.md) for comprehensive firmware build, flash, and test commands.

**Quick reference:**
```bash
make cf2_defconfig && make -j$(nproc)   # Build for Crazyflie 2.x
make unit                                # Run all unit tests (Unity/CMock)
make unit FILES=test/utils/src/test_num.c  # Run single test file
make test_python                         # Python binding tests (pytest)
```

**Docker (matches CI):**
```bash
docker run --rm -v ${PWD}:/module bitcraze/builder bash -c "make cf2_defconfig && ./tools/build/build"
```

Output goes to `build/` (cf2.bin, cf2.elf, etc.).

## Architecture

FreeRTOS-based embedded firmware for STM32 (Crazyflie drones). Kbuild config system.

**Control pipeline:** Sensors → Estimator → Commander → Controller → Power Distribution → Motors

| Directory | Purpose |
|-----------|---------|
| `src/modules/` | Core firmware: control, estimation, communication, logging, params |
| `src/drivers/` | Low-level hardware drivers (IMU, baro, radio, flow) |
| `src/deck/` | Expansion deck plugin drivers |
| `src/hal/` | Hardware Abstraction Layer |
| `src/platform/` | Platform-specific code (cf2, bolt, tag, flapper) |
| `test/` | Unit tests (CMock/Unity) |
| `examples/` | App layer examples |
| `docs/` | Full developer docs (estimators, controllers, CRTP, etc.) |

See [docs/](../docs/) for detailed subsystem docs: [estimators](../docs/functional-areas/sensor-to-control/state_estimators.md), [controllers](../docs/functional-areas/sensor-to-control/controllers.md), [CRTP protocol](../docs/functional-areas/crtp/), [app layer](../docs/userguides/app-layer.md).

## Code Style

- **C:** GCC `-Werror` on release. Debug adds `-Wconversion`. No specific formatter enforced.
- **Python:** `flake8` with `max-line-length: 120` (see `.flake8`).

## Python Research Scripts

Root-level Python scripts implement EKF replay, tuning, and relative localization research.

**Environment:** Python 3 virtualenv at `~/flapper/bin/activate`. Key dependencies: numpy, pandas, scipy, matplotlib, optuna.

**Data pipeline pattern:**
1. Binary SD logs → CSV via `usdlog_to_csv.py` (uses `cfusdlog` from `tools/usdlog/`)
2. Decode + sync with OptiTrack mocap ground truth (cross-correlation on height)
3. Replay through reimplemented EKF → metrics and plots

**Key scripts:**
| Script | Purpose |
|--------|---------|
| `ekf_utils.py` | Generic EKF class (numerical Jacobians, RK4 discretization) |
| `ekf_replay.py` | Replay flight data through Kalman filter reimplementation |
| `ekf_tune.py` | Optuna hyperparameter tuning (200+ trials) |
| `ekf_ablation.py` | Feature ablation + sensitivity analysis |
| `process_day02_data.py` | Day 02 data pipeline: decode → sync → body-frame velocities |
| `run_day02_ekf.py` | Relative 3-state EKF on day_02 data, all 6 drone pair directions |
| `generate_beacon_data.py` | Synthetic beacon data for `DataFlapperEKF/` CSVs |

**Frame convention:** cf_x = optitrack_z, cf_y = optitrack_x, cf_z = optitrack_y.

**Units:** `stateEstimateZ.rateYaw` is millirad/s (divide by 1000 for rad/s).

**Drone 0** is a static ground beacon (zeroed velocity, yaw rate, height).

## Conventions

- **Submodules:** Clone with `--recursive` or run `git submodule update --init --recursive`.
- **Feature overlays:** Stack `.conf` files from `configs/` on defconfigs via `merge_config.sh`.
- **Logging/Param macros:** Use `LOG_GROUP_START`/`LOG_ADD`/`LOG_GROUP_STOP` and `PARAM_*` equivalents. `_CORE` variants mark essential variables.
- **App layer:** Define `appMain()` (FreeRTOS task) or `appInit()` when `CONFIG_APP_ENABLE=y`. See `examples/`.
- **CI quirk:** Unit tests may fail locally with TSan/ASLR errors → `sudo sysctl vm.mmap_rnd_bits=28` or use Docker.
