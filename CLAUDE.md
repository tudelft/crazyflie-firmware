# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

**Configure for a target platform (required before building):**
```bash
make cf2_defconfig      # Crazyflie 2.0/2.1 (most common)
make cf21bl_defconfig   # Crazyflie 2.1 Brushless
make bolt_defconfig     # Bolt platform
make tag_defconfig      # Roadrunner/TAG platform
make flapper_defconfig  # Flapper Nimble+
```

**Build:**
```bash
make -j$(nproc)         # Linux
make -j$(sysctl -n hw.ncpu)  # macOS
```

**Other build targets:**
```bash
make size               # Show FLASH/RAM/CCM memory usage
make clean              # Clean compiled files
make menuconfig         # Interactive Kbuild configuration UI
```

**Flash:**
```bash
make cload              # Via Crazyradio (radio bootloader)
make flash              # Via OpenOCD/ST-Link debug adapter
make flash_dfu          # Via USB DFU (auto-bootloader entry)
```

Output artifacts go to `build/` (cf2.bin, cf2.elf, cf2.hex, cf2.dfu).

**Docker build (matches CI):**
```bash
docker run --rm -v ${PWD}:/module bitcraze/builder bash -c "make cf2_defconfig && ./tools/build/build"
```
`tools/build/build` runs unit tests + Python tests + firmware build + ELF checks (same as CI).

**Out-of-tree app build:**
```bash
cd examples/app_hello_world && make -j$(nproc)
```
App examples have their own `Makefile` that includes `tools/make/oot.mk` and sets `OOT_CONFIG` for app-layer Kconfig options. Each app directory needs a `Kbuild`, `app-config`, and `Makefile` pointing back to `CRAZYFLIE_BASE`.

## Testing

```bash
make unit                                          # Run all unit tests
make unit FILES=test/utils/src/test_num.c          # Run a specific test file
make test_python                                   # Run Python binding tests (pytest)
```

Unit tests use CMock/Unity/Rake with AddressSanitizer. If tests fail with TSan/ASLR errors:
```bash
sudo sysctl vm.mmap_rnd_bits=28
```

The CI pipeline (`bitcraze/builder` Docker image) avoids these local tool issues.

## Code Style

- **C:** GCC with `-Werror` on release builds (warnings are errors). Debug builds add `-Wconversion`.
- **Python:** `flake8` with max line length 120 (see `.flake8`).

## Architecture Overview

The firmware is a **FreeRTOS-based embedded system** targeting STM32 microcontrollers. It uses a Kbuild configuration system (similar to the Linux kernel) to gate features.

### Startup Flow
`src/init/main.c` → bootloader check → platform init → `src/modules/src/system.c` (system task) → FreeRTOS scheduler starts all subsystem tasks.

### Core Control Pipeline
```
Sensors → Estimator → Commander → Controller → Power Distribution → Motors
```
- **Stabilizer** (`src/modules/src/stabilizer.c`): The main control loop task. Reads sensor data, calls the active estimator, then the active controller, and outputs motor commands.
- **Estimators** (`src/modules/src/estimator/`): Kalman (default), UKF, or Complementary. The Kalman filter core lives in `src/modules/src/kalman_core/`.
- **Controllers** (`src/modules/src/controller/`): PID (default), Mellinger, Lee, Brescianini.
- **Commander** (`src/modules/src/commander.c`): Processes setpoints from radio, high-level trajectory planner, or the app layer.
- **Power distribution** (`src/modules/src/power_distribution*.c`): Allocates thrust to individual motors.

### Communication Stack
- **CRTP** (Crazyflie Real-Time Protocol): the main host↔drone protocol, multiplexed over radio (Crazyradio), BLE, USB, or UART.
- `src/modules/src/comm.c` initializes all CRTP links.
- **Logging** (`src/modules/src/log.c`) and **Parameters** (`src/modules/src/param.c`) are exposed via CRTP and use macros for automatic registration. Pattern: `LOG_GROUP_START(name)` / `LOG_ADD(type, name, &var)` / `LOG_GROUP_STOP(name)`, and similarly `PARAM_GROUP_START` / `PARAM_ADD` / `PARAM_GROUP_STOP`. The `_CORE` variants mark variables as essential (always streamed).

### Expansion Deck System
`src/deck/` implements a plugin architecture for hardware expansions (Flow deck, Lighthouse, Loco positioning, BigQuad, etc.). Each deck driver registers itself with metadata (GPIO usage, init/test functions) and is auto-detected at runtime via I2C memory.

### Key Directories
| Directory | Purpose |
|-----------|---------|
| `src/init/` | Entry point (`main.c`) |
| `src/modules/` | Core firmware (control, estimation, communication, logging, params) |
| `src/drivers/` | Low-level hardware drivers (IMU, barometer, radio, flow cam) |
| `src/deck/` | Expansion deck drivers and core |
| `src/hal/` | Hardware Abstraction Layer |
| `src/platform/` | Platform-specific code (cf2, bolt, tag, flapper) |
| `src/utils/` | Math, memory management, key-value store |
| `src/lib/` | Vendored libraries (FreeRTOS, CMSIS) |
| `configs/` | Kbuild platform defconfigs and feature overlay configs |
| `test/` | Unit tests (CMock/Unity) |
| `examples/` | App layer usage examples |
| `docs/` | Developer and user documentation |

### App Layer
When `CONFIG_APP_ENABLE=y`, user code can define `appMain()` (runs as a FreeRTOS task) or `appInit()` (called during init). This is the intended extension point for custom firmware. See `examples/` and `docs/userguides/app-layer.md`.

### Platform Configuration
The Kbuild system (see `Kconfig`, `configs/`) gates features at compile time. Feature overlays like `configs/bosch.conf`, `configs/loco_tdoa3.conf` can be stacked on top of a defconfig. `make allyesconfig` / `allnoconfig` / `randconfig` are used in CI to test configuration coverage.

## Submodules

The repo uses git submodules for vendored libraries (`vendor/`, `src/lib/`). Clone with `--recursive` or run `git submodule update --init --recursive` after cloning.
