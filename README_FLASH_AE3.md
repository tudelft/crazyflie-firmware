# Flashing the IMAV2026 flapper firmware (flow + USD + AE3)

This is the flow-only flapper build with the tuned EKF, the ToF innovation gate,
and the **AE3 forward-camera link** (`ae3.dist` log variable). Config:
[`configs/flapper_ae3_defconfig`](configs/flapper_ae3_defconfig).

## Flash steps

Run these from a terminal:

```bash
cd ~/crazyflie-firmware

make clean                    # wipe old build artifacts
make flapper_ae3_defconfig    # select the flow + USD + AE3 config (CPPM off)
make -j$(nproc)               # build, using all CPU cores (fast)
make cload                    # flash over the Crazyradio
```

**Before `make cload`:** put the flapper into the radio bootloader —
**hold the power button until the blue LED starts flashing**, then release.
Have the **Crazyradio plugged into the laptop** and the drone in range.
`make cload` will find the bootloader and write the firmware.

> Close **cfclient** first — it holds the Crazyradio and `make cload` will fail
> while it is connected.

## After flashing — re-apply the tuned params ⚠️

Reflashing does **not** clear the EEPROM, so the drone keeps whatever EKF/PID
values were persisted before. Push the tuned set back:

```bash
python set_ekf_params.py            # set + persist the tuned EKF params
python set_ekf_params.py --dry-run  # read them back to verify
python reset_pid_gains.py           # clear persisted PID gains, then POWER-CYCLE
```

## Telemetry — verify the link is alive

Open **cfclient** (or your Python log script) and watch these log variables:

**AE3 forward camera** (`ae3` group, from [ae3deck.c](src/deck/drivers/src/ae3deck.c)):

| variable   | meaning |
|------------|---------|
| `ae3.dist` | median ToF distance from the AE3 (metres). `nan` = sensor got no reading — treat as "range unknown, hold", never as "clear ahead" (test `d != d` in Python). |
| `ae3.rx`   | valid frames received. **Climbing ~10/s = the link is alive.** Stuck at 0 = wiring/bus wrong. |
| `ae3.age`  | ms since the last frame. **> ~300 = the link is DOWN.** |
| `ae3.bad`  | checksum failures. Climbing = noise or a baud mismatch. |

**ToF height gate** (`tofGate` group, from [mm_tof.c](src/modules/src/kalman_core/mm_tof.c)):

| variable        | meaning |
|-----------------|---------|
| `tofGate.gated` | 1 = the last ToF update was rejected (crossing a gate/window sill). |
| `tofGate.innov` | last innovation (measured − predicted), m — use to tune `tofGate.gate`. |
| `tofGate.rej`   | consecutive rejects. |
| `tofGate.ref`   | last accepted ToF reading (the reference), m. |

To disable the height gate live (no reflash): set param `tofGate.gate = 0`.

## AE3 camera side (not this repo)

The AE3 runs `main.py` (OpenMV). It must match your wiring:

- **`UART_BUS = 5`** for the P2/P3 wiring: AE3 **P2 (UART5 TX) → CF PA3 (UART2 RX)**,
  AE3 **P3 (UART5 RX) → CF PA2 (UART2 TX)**, **GND → GND**.
- Baud 115200, frame `0xAE 0x51 | float32 LE | xor(4 bytes)` — matches
  `ae3deck.c`.

## Build without the AE3 (flow + USD only)

If the AE3 is not fitted, swap one line:

```bash
make clean
make flapper_ekf_defconfig    # tuned EKF + ToF gate, no AE3 deck
make -j$(nproc)
make cload
```
