# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.x and the Roadrunner.

## IMAV2026 flapper build — flash steps

Flow + USD + AE3 forward camera (config `configs/flapper_ae3_defconfig`).

```bash
cd ~/crazyflie-firmware
make clean
make flapper_ae3_defconfig
make -j$(nproc)
make cload            # hold the flapper's power button until the blue LED flashes, then release
```

`make cload` flashes over the Crazyradio — plug the radio in, close cfclient
first, and put the drone in the bootloader (hold power → blue LED flashing).

**After flashing, re-apply the persisted params** (reflashing keeps stale EEPROM):

```bash
python set_ekf_params.py            # tuned EKF params
python reset_pid_gains.py           # default PID gains, then POWER-CYCLE
```

**Telemetry to verify the link** (cfclient or a Python log script):

- `ae3.dist` — AE3 median ToF distance (m); `nan` = no reading, hold.
- `ae3.rx` — valid frames; climbing ~10/s = link alive. Stuck at 0 = wiring/bus wrong.
- `ae3.age` — ms since last frame; > ~300 = link down.
- `tofGate.gated` / `tofGate.innov` — ToF height gate (set param `tofGate.gate = 0` to disable live).

AE3 camera side (`main.py` on the OpenMV board): `UART_BUS = 5` for the P2/P3
wiring — AE3 P2 (UART5 TX) → CF PA3 (UART2 RX), AE3 P3 → CF PA2, GND → GND.

For a build without the AE3 (flow + USD only): `make flapper_ekf_defconfig` instead.

### Flapper Nimble Triple

Triple support is integrated from `1048bcab7a955438d6d990d4ac0f7b4ad5f729f7`.
Use `make flapper3_ae3_defconfig` for the IMAV2026 MTF-02 + USD + AE3 setup,
then `make -j$(nproc)`. The output is `build/flapper3.bin`.
This config uses the revC-and-newer mapping with yaw servo inversion enabled;
CPPM is disabled to free UART2 RX.

The original Triple configs are also available:

| Config | M1 | M2 | M3 | M4 | Servo inverted |
| --- | --- | --- | --- | --- | --- |
| `flapper3_defconfig` | Left | Yaw servo | Right | Rear | No |
| `flapper3_revA_defconfig` | Left | Yaw servo | Right | Rear | Yes |
| `flapper3_revB_defconfig` | Left | Rear | Yaw servo | Right | Yes |

The default and revB configs force USD + LED ring; revA forces no decks.
They do not enable MTF-02 or AE3. The IMAV2026 `flapper3_ae3_defconfig` already
enables servo inversion for revA/revB. For revB, also enable the Triple PCB
revision option under power distribution using `make menuconfig`.

Integration checks before flight:

- Confirm motor order, yaw direction and IMU orientation on the actual airframe.
  The imported Triple alignment is `(0, -90, 180)` degrees for every revision;
  the older Nimble+ revB uses a different pitch alignment (`+90` degrees).
  Keep the imported Triple setting until its mounting is verified.
- Use the PID controller: the Triple mixer only supports legacy control outputs.
  Clear persisted PID gains with `reset_pid_gains.py` and power-cycle when
  switching from Nimble+ so the Triple defaults take effect.
- Triple EKF drag defaults use `BX = BY = 0.7 * Flapper BX = 3.076276` and
  `BZ = Flapper BZ = 0.0611769`. Drag-center offsets remain zero and noise
  defaults are generic. These initial settings still need Triple validation;
  do not automatically apply `set_ekf_params.py` from the instructions above.
  Previously persisted `kalman.*` parameters can override firmware defaults.
- Trim parameters are in `flapper3.*`. `flapper3.flapperMaxThrust` limits
  collective thrust before mixing, not each motor's final output.

Firmware builds and mixer checks do not establish flight stability. Triple PID,
EKF tuning and the deck installation still need validation on the hardware.

### Crazyflie 1.0 support

The 2017.06 release was the last release with Crazyflie 1.0 support. If you want
to play with the Crazyflie 1.0 and modify the code, please clone this repo and
branch off from the 2017.06 tag.

## Building and Flashing
See the [building and flashing instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) in the github docs folder.


## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on our website.

## Generated documentation

The easiest way to generate the API documentation is to use the [toolbelt](https://github.com/bitcraze/toolbelt)

```tb build-docs```

and to view it in a web page

```tb docs```

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution

To run the tests please have a look at the [unit test documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/unit_testing/).

## License

The code is licensed under LGPL-3.0
