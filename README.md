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
