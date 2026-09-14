# motor_control

An open-source motor-control platform for integrated joints and servo actuators. Built around an HC32F4-series MCU, the current firmware implements three-phase BLDC/PMSM field-oriented control (FOC), MA732 magnetic-encoder sampling, MIT-style position/velocity/feed-forward torque control, CAN and UART command interfaces, and persistent multi-turn position tracking at the reducer output.

> [!WARNING]
> Motor drives involve high currents, power electronics, and rapidly moving mechanisms. Disconnect the load, use a current-limited supply, and keep an emergency stop available during initial testing. The default parameters only describe the author's current prototype and must not be treated as safe values for another motor or power stage.

## Project Status

This repository is being organized into a complete open-source release covering electronics, mechanics, assembly, and commissioning.

| Module | Status | Location |
| --- | --- | --- |
| Motor-control firmware | Published; being refined | [`source/`](source/) |
| Python CAN tools | Published | [`can_trans.py`](can_trans.py) |
| Control theory and parameters | Published | [`docs/control/`](docs/control/) |
| CAN and UART protocols | Documented | [`docs/protocol/`](docs/protocol/) |
| Schematics and PCB | To be released | [`hardware/electronics/`](hardware/electronics/) |
| Second-encoder design | To be released | [`hardware/electronics/encoder-2/`](hardware/electronics/encoder-2/) |
| Mechanical STEP models | Published; manufacturing details in progress | [`hardware/mechanical/`](hardware/mechanical/) |
| Preliminary mechanical BOM | Published | [`hardware/bom/`](hardware/bom/) |
| CAD renders and test evidence | Renders published; prototype tests pending | [`results/`](results/) |
| Assembly and tuning | Structure available; content in progress | [`docs/assembly/`](docs/assembly/), [`docs/tuning/`](docs/tuning/) |

The current repository is a source snapshot. It does not yet include a complete IDE project, device startup code, linker script, or the HC32 Device Driver Library (DDL), so it cannot be built as a standalone project until those dependencies are added.

## Features

- Three-phase current sensing, Clarke/Park transforms, dq current PI control, and SVPWM
- 20 kHz center-aligned PWM; current loop executed at one-quarter of the fast-loop rate
- MA732 14-bit magnetic encoder over SPI3 with DMA-assisted acquisition
- MIT-style control combining position error, velocity error, and feed-forward torque
- Output-angle conversion for a 57:7 reduction ratio and flash-backed multi-turn position tracking
- Classic CAN 2.0 commands, acknowledgements, heartbeat messages, and bus diagnostics
- UART command input and runtime telemetry
- ADC-offset calibration, phase-current protection, three-phase sum checks, and encoder-stale protection

## Control Architecture

```text
Position / velocity / feed-forward torque target
                         │
                         ▼
τout = Kp·position_error + Kd·velocity_error + τff
                         │  limits and static-friction compensation
                         ▼
τmotor = τout / (gear ratio × transmission efficiency)
                         │
                         ▼
Iq_ref = τmotor / Kt, Id_ref = 0
                         │
                         ▼
dq current PI → inverse Park → SVPWM → inverter → motor
                         ▲
                         └──── phase currents + MA732 rotor angle
```

The main configuration parameters are in [`source/motor/config.h`](source/motor/config.h). See [`docs/control/mit-control.md`](docs/control/mit-control.md) for the current control notes and test procedure.

## Repository Layout

```text
motor_control/
├─ source/                     # MCU firmware source
│  ├─ inc/                    # Peripheral interface headers
│  ├─ src/                    # ADC/CAN/DMA/GPIO/SPI/TIM/UART drivers
│  └─ motor/                  # FOC, controller, encoder, position memory
├─ tools/                      # Host-tool documentation and future tools
├─ docs/                       # Control, protocol, assembly, and tuning docs
├─ hardware/                   # Electronics, second encoder, mechanics, and BOM
├─ results/                    # CAD renders, prototype photos, and experiments
├─ firmware/projects/keil/     # Reserved for the reproducible Keil project
├─ can_trans.py               # Damiao USB-CAN command-line utility
├─ can_code_control_test.py   # Position-sequence test
└─ vla_can_control_example.py # Example VLA-to-CAN integration
```

See [`docs/README.md`](docs/README.md) for the documentation index.

Small, redistributable HC32F448 J-Link Flash Algorithms are included under [`tools/programming/`](tools/programming/). UART and CAN tool settings and vendor download links are documented under [`tools/uart/`](tools/uart/) and [`tools/can/`](tools/can/); third-party proprietary executables are not mirrored.

## Quick Start

For the complete hardware/software checklist, debugger choices, XHSC packages, and dependency-distribution policy, read [`docs/development-setup.md`](docs/development-setup.md).

### 1. Prepare the Firmware Project

The repository currently contains application source only. The empty [`firmware/projects/keil/`](firmware/projects/keil/) directory is reserved for a tested, directly buildable Keil project. Start with the official `HC32F448_DDL_Rev1.3.0`, `HC32F448_Template_Rev1.1.0`, and `HC32F448_IDE_Rev1.1.0` packages from the [HC32F448 product page](https://www.xhsc.com.cn/product/1213.html). Add `source/inc/`, `source/src/*.c`, and `source/motor/*.c` together with the correct startup code, linker script, CMSIS files, HC32 DDL, and math-library support.

### 2. Review the Configuration

Before applying power, check at least the following settings in [`source/motor/config.h`](source/motor/config.h):

- Motor pole pairs, encoder direction, and electrical zero offset
- Reduction ratio, transmission efficiency, and torque constant `Kt`
- Current-sense scaling, polarity, and overcurrent thresholds
- PWM frequency, dead time, voltage limits, and current limits
- CAN bit timing and transceiver standby/enable configuration
- Whether internal-flash sector 31 overlaps the application or other stored data

`APP_ALLOW_OPEN_LOOP_RUN` is `1` by default, which allows the startup sequence to proceed automatically after ADC-offset calibration. Set it to `0` during a new hardware port until all static checks have passed.

### 3. Use the Python CAN Tool

```bash
python -m pip install -r requirements.txt
python can_trans.py --port COM5 --serial-baud 921600 --pos 10 --vel 0 --tau 0
```

Interactive or repeated transmission:

```bash
python can_trans.py --port COM5 --interactive
python can_trans.py --port COM5 --pos 10 --repeat --hz 20
```

The current host utility targets the serial protocol used by a Damiao USB-CAN adapter; it is not a generic `python-can` driver. See [`docs/protocol/can.md`](docs/protocol/can.md) for payload units and frame layout.

### 4. Perform a Small-Angle No-Load Test

After checking wiring, current limiting, phase order, and encoder direction, wait until the state reaches `st=2`, then send these UART commands:

```text
m0
m10
m30
m0
```

Immediately remove power if the motor kicks, runs in the wrong direction, oscillates, draws sustained high current, or reports discontinuous position. Continue troubleshooting with [`docs/tuning/README.md`](docs/tuning/README.md).

## Default Parameters

These values are a reading aid only. [`source/motor/config.h`](source/motor/config.h) is authoritative.

| Parameter | Current default |
| --- | --- |
| PWM / fast-control frequency | 20 kHz |
| dq current-loop frequency | 5 kHz |
| MIT outer-loop frequency | 1 kHz |
| Motor pole pairs | 14 |
| Reduction ratio (motor:output) | 57:7 |
| Encoder | MA732, 14 bit, SPI mode 3 |
| Output-torque / `Iq` limit | 0.020 N·m / 0.120 A |
| CAN command / response ID | `0x201` / `0x202` |
| Nominal CAN rate | Timing comments correspond to 1 Mbit/s |

## Contributing

Issues and pull requests are welcome. Read [`CONTRIBUTING.md`](CONTRIBUTING.md) first and include the hardware revision, motor and encoder, supply conditions, and reproducible steps. Changes to the power stage or control parameters should also state the tested operating envelope and relevant safety limits.

## License

An open-source license has not yet been selected. Until a `LICENSE` file is added, the repository contents remain copyright-protected; public visibility alone does not grant permission to copy, modify, or redistribute them. Electronics, mechanical designs, and third-party libraries may ultimately require separate license notices.

## Contact and Citation

- Project: <https://github.com/ZJYSII/motor_control>
- Questions and suggestions: please use GitHub Issues

If this project helps your research or development, consider starring it. Citation metadata will be added with the first stable release.
