# OpenEAI-FOC

Open-source integrated-joint hardware and field-oriented motor control for embodied AI.

![OpenEAI-FOC overview](assets/images/openeai-foc-overview.png)

OpenEAI-FOC combines a compact motor and reducer, custom drive electronics, dual-encoder-ready sensing, HC32F448 firmware, and host-side control tools. The current firmware implements three-phase BLDC/PMSM field-oriented control (FOC), MA732 magnetic-encoder sampling, MIT-style position/velocity/feed-forward torque control, CAN and UART command interfaces, and flash-backed multi-turn output-position tracking.

The project is a successor to the low-level actuator work developed for [OpenEAI-Arm](https://github.com/eai-yeslab/OpenEAI-Arm). It inherits the OpenEAI goal of reproducible embodied-AI hardware and is intended to serve as a joint-level hardware and control component for both OpenEAI-Arm and [OpenEAI-VLA](https://github.com/eai-yeslab/OpenEAI-VLA). Its longer-term direction is mechanics-aware force/torque FOC and joint optimization of learned VLA policies, actuator dynamics, sensing, and safety constraints.

> [!WARNING]
> Motor drives involve high currents, power electronics, and rapidly moving mechanisms. Disconnect the load, use a current-limited supply, and keep an emergency stop available during initial testing. The default parameters describe one prototype only and must not be treated as safe values for another motor, power stage, or mechanism.

## OpenEAI System Context

```text
OpenEAI-VLA
  perception, reasoning, and action policy
                    │ action targets / learned feed-forward
                    ▼
OpenEAI-Arm
  robot coordination, kinematics, task execution, and system safety
                    │ joint commands
                    ▼
OpenEAI-FOC
  joint electronics, FOC, dual-encoder sensing, CAN/UART, and protection
                    │ phase voltage and mechanical output
                    ▼
Motor + reducer + joint
                    │ current, rotor position, output position, temperature
                    └──────────────────────── feedback ────────────────────┘
```

Safety-critical limits and protection must remain enforceable at the actuator level, independently of the learned policy.

## What Is Included

| Module | Status | Location |
| --- | --- | --- |
| Motor-control source | Published; being refined | [`source/`](source/) |
| Python CAN tools and examples | Published | [`can_trans.py`](can_trans.py), [`vla_can_control_example.py`](vla_can_control_example.py) |
| Control theory and parameters | Published | [`docs/control/`](docs/control/) |
| CAN and UART protocols | Documented | [`docs/protocol/`](docs/protocol/) |
| Mechanical STEP models | Published | [`hardware/mechanical/`](hardware/mechanical/) |
| Preliminary motor BOM | Published and explained | [`hardware/bom/`](hardware/bom/) |
| Images and control demonstrations | Published | [`assets/`](assets/) |
| Schematics, PCB, and fabrication files | Planned release | [`hardware/electronics/`](hardware/electronics/) |
| Second-encoder electronics | Planned release | [`hardware/electronics/encoder-2/`](hardware/electronics/encoder-2/) |
| Reproducible Keil project | Reserved; not yet published | [`firmware/projects/keil/`](firmware/projects/keil/) |
| Assembly, commissioning, and tuning data | In progress | [`docs/assembly/`](docs/assembly/), [`docs/tuning/`](docs/tuning/) |
| Quantitative experiment data | Initial demonstrations published; instrumented datasets planned | [`results/`](results/) |

The repository is currently a source snapshot. It does not yet contain the complete Keil project, startup files, linker configuration, or HC32 Device Driver Library (DDL), and therefore cannot yet be built standalone.

## Current Results

The overview image at the top of this page shows the assembled physical prototype, the integrated-joint CAD and exploded assembly, the custom circular drive board, and the second-encoder board. Additional prototype photographs, exploded views, and short control demonstrations are available in [`assets/`](assets/). The following status distinguishes working demonstrations from measurements that still need to be published.

| Capability or evidence | Current status |
| --- | --- |
| Physical integrated-joint prototype | Assembled and photographed |
| Mechanical assembly and exploded model | Published as STEP models and renders |
| Motor-side encoder closed-loop FOC | Implemented in firmware |
| MIT position, velocity, and feed-forward torque interface | Implemented over UART and CAN |
| Flash-backed reducer-output multi-turn tracking | Implemented; limitations noted in the FAQ |
| OpenEAI-VLA-to-CAN integration example | Published |
| Drive PCB and second-encoder board | Shown in the project overview; design files pending |
| MIT mode and position-command demonstrations | Published as videos |
| Velocity command at 1.2 rad/s | Published as a demonstration video |
| Torque command at 5 N·cm | Published as a demonstration video; calibrated measurement data pending |
| Fault-protection behavior | Published as a demonstration video; formal fault matrix pending |
| Position-tracking accuracy and bandwidth | Quantitative dataset pending |
| Calibrated output torque and force-control performance | Pending |
| Thermal, efficiency, backlash, and load testing | Pending |

No numerical performance claim should be inferred from the current photographs or source release. Reproducible plots, raw logs, test conditions, and scripts will be added under [`results/experiments/`](results/experiments/).

## Control Architecture

```text
Position / velocity / feed-forward torque target
                         │
                         ▼
τout = Kp·position_error + Kd·velocity_error + τff
                         │ limits and static-friction compensation
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

Current features include:

- Three-phase current sensing, Clarke/Park transforms, dq current PI control, and SVPWM
- 20 kHz center-aligned PWM and a 5 kHz current loop
- MA732 14-bit magnetic encoder over SPI3 with DMA-assisted acquisition
- 1 kHz MIT-style outer control combining position, velocity, and feed-forward torque
- Output-angle conversion for a 57:7 reduction ratio
- Classic CAN 2.0 commands, acknowledgements, heartbeat messages, and bus diagnostics
- UART command input and runtime telemetry
- ADC-offset calibration, phase-current protection, three-phase sum checks, and encoder-stale protection

See [`docs/control/mit-control.md`](docs/control/mit-control.md) for the current control notes and test procedure.

## Hardware and Software Requirements

### Core hardware

- OpenEAI-FOC motor, reducer, enclosure, bearings, fasteners, and magnetic components listed in [`hardware/bom/`](hardware/bom/)
- HC32F448-based drive board, three-phase inverter, phase-current sensing, gate drive, and CAN transceiver
- MA732 motor-side magnetic encoder; the second output-side encoder is part of the planned dual-encoder release
- Current-limited DC supply, emergency stop, suitable cabling, and a mechanically safe test fixture
- J-Link debugger connected through SWD; a dedicated XHSC programmer is not required for the documented workflow
- USB-to-UART or supported USB-CAN adapter for commissioning and command testing

### Development software

- Keil MDK-ARM
- Official HC32F448 DDL, template, and IDE support packages from the [XHSC HC32F448 product page](https://www.xhsc.com.cn/product/1213.html)
- XHCode, optional, for peripheral configuration and project generation
- SEGGER J-Link software with the repository's HC32 device support additions
- Python 3 for the supplied host tools
- A serial terminal such as XCOM, or an equivalent terminal; vendor download links and tested settings are documented rather than redistributing proprietary executables

Small redistributable HC32F448 J-Link flash algorithms and device descriptions are included under [`tools/programming/jlink/hc32f448/`](tools/programming/jlink/hc32f448/). Follow its [J-Link setup instructions](tools/programming/jlink/hc32f448/README.md) to add the HC32F448 device to J-Link. See [`docs/development-setup.md`](docs/development-setup.md) for the complete dependency and programming-tool policy.

## Configuration

The main firmware configuration is [`source/motor/config.h`](source/motor/config.h). Review every hardware-dependent value before energizing a new board.

| Parameter | Current prototype default |
| --- | --- |
| PWM / fast-control frequency | 20 kHz |
| dq current-loop frequency | 5 kHz |
| MIT outer-loop frequency | 1 kHz |
| Motor pole pairs | 14 |
| Reduction ratio, motor:output | 57:7 |
| Motor-side encoder | MA732, 14 bit, SPI mode 3 |
| Output-torque / `Iq` limit | 0.020 N·m / 0.120 A |
| CAN command / response ID | `0x201` / `0x202` |
| Nominal CAN rate | Timing comments correspond to 1 Mbit/s |

At minimum, verify:

- Motor pole pairs, phase order, encoder direction, and electrical zero offset
- Reduction ratio, transmission efficiency, torque constant `Kt`, and current-to-torque calibration
- Current-sense scaling and polarity, overcurrent thresholds, and supply-voltage limits
- PWM frequency, dead time, current limits, and gate-driver behavior
- CAN bit timing, node IDs, and transceiver standby/enable polarity
- Internal-flash sector allocation for persistent position data

`APP_ALLOW_OPEN_LOOP_RUN` is `1` in the current snapshot, allowing startup to continue after ADC-offset calibration. Set it to `0` for a new hardware port until the static checks, current sensing, phase order, and encoder direction have been validated.

## Repository Layout

```text
OpenEAI-FOC/
├─ source/                     # HC32 firmware source
│  ├─ inc/                    # Peripheral interface headers
│  ├─ src/                    # ADC/CAN/DMA/GPIO/SPI/TIM/UART drivers
│  └─ motor/                  # FOC, controller, encoder, position memory
├─ firmware/projects/keil/     # Reserved for the reproducible Keil project
├─ hardware/
│  ├─ electronics/            # Drive PCB and second-encoder release structure
│  ├─ mechanical/             # STEP models and mechanical documentation
│  └─ bom/                    # BOM source and explanation
├─ docs/                       # Human-readable control, protocol, assembly, setup, and tuning guides
├─ tools/                      # Programs, debugger support files, and CAN/UART utilities
├─ assets/
│  ├─ images/                 # Overview, photographs, and CAD renderings
│  └─ videos/                 # Control and protection demonstrations
├─ results/
│  └─ experiments/            # Reproducible measurements and datasets
├─ can_trans.py                # Damiao USB-CAN command-line utility
├─ can_code_control_test.py    # Position-sequence test
└─ vla_can_control_example.py  # Example VLA-to-CAN integration
```

See [`docs/README.md`](docs/README.md) for the documentation index.

## Quick Start

### 1. Clone the repository and fetch large assets

```bash
git clone https://github.com/ZJYSII/OpenEAI-FOC.git
cd OpenEAI-FOC
git lfs install
git lfs pull
```

### 2. Prepare the firmware project

Until the tested Keil project is published, begin with the official `HC32F448_DDL_Rev1.3.0`, `HC32F448_Template_Rev1.1.0`, and `HC32F448_IDE_Rev1.1.0` packages from XHSC. Add `source/inc/`, `source/src/*.c`, and `source/motor/*.c` to the matching project together with the correct startup code, linker script, CMSIS files, HC32 DDL, and math-library support.

XHCode may be used to generate the initial peripheral project, but it is not a substitute for a version-controlled, known-good Keil project. Publishing both the application source and a directly buildable Keil project is the target release format.

### 3. Configure and program the controller

Review [`source/motor/config.h`](source/motor/config.h), begin with the motor mechanically unloaded, and use a current-limited power supply. A J-Link over SWD is sufficient after completing the HC32 device-registration steps in the [HC32F448 J-Link guide](tools/programming/jlink/hc32f448/README.md).

### 4. Install and use the Python CAN tool

```bash
python -m pip install -r requirements.txt
python can_trans.py --port COM5 --serial-baud 921600 --pos 10 --vel 0 --tau 0
```

Interactive or repeated transmission:

```bash
python can_trans.py --port COM5 --interactive
python can_trans.py --port COM5 --pos 10 --repeat --hz 20
```

The utility currently targets the serial protocol used by a Damiao USB-CAN adapter; it is not a generic `python-can` interface. See [`docs/protocol/can.md`](docs/protocol/can.md) for frame layout and units.

### 5. Perform a small-angle, no-load test

After checking wiring, current limiting, phase order, and encoder direction, wait until the state reaches `st=2`, then send:

```text
m0
m10
m30
m0
```

Immediately remove power if the motor kicks, turns in the wrong direction, oscillates, draws sustained high current, or reports discontinuous position. Continue with [`docs/tuning/README.md`](docs/tuning/README.md).

## Roadmap and TODO

### Release completeness

- [x] Publish the application firmware source
- [x] Publish CAN/UART protocol notes and Python control examples
- [x] Publish the current mechanical STEP package and preliminary BOM
- [x] Add the system overview, CAD renders, prototype imagery, and initial control videos
- [ ] Publish a tested, directly buildable Keil project with pinned HC32 dependencies
- [ ] Release drive-board schematics, PCB source, Gerbers, and manufacturing outputs
- [ ] Release the second-encoder schematic, PCB, calibration procedure, and firmware interface
- [ ] Add complete assembly, wiring, commissioning, and tuning procedures
- [ ] Add machine-readable BOM and sourcing alternatives
- [ ] Select licenses for firmware, electronics, mechanics, and documentation

### Validation and control

- [ ] Publish raw logs and reproducible plots for current-loop bandwidth and position tracking
- [ ] Calibrate motor torque constant, reducer efficiency, friction, backlash, and output torque
- [ ] Add thermal, endurance, efficiency, and loaded-motion experiments
- [ ] Fuse motor-side and output-side encoders for backlash, compliance, and disturbance estimation
- [ ] Develop mechanics-aware force/torque FOC with friction and transmission compensation
- [ ] Add torque, impedance, admittance, and contact-safe control modes
- [ ] Validate protection behavior for overcurrent, sensor loss, bus loss, stall, and overheating

### OpenEAI integration

- [ ] Define a stable OpenEAI-Arm joint API and multi-axis CAN addressing scheme
- [ ] Add synchronized telemetry for current, estimated torque, both encoders, temperature, and faults
- [ ] Build datasets that align VLA actions with actuator state and contact measurements
- [ ] Explore latency-aware VLA and actuator co-design, learned feed-forward, and residual control
- [ ] Jointly optimize OpenEAI-VLA policies and OpenEAI-FOC dynamics while preserving independent hard safety limits

## Frequently Asked Questions

### Can the firmware be built directly after cloning?

Not yet. The application source is available, but the complete Keil project, startup files, linker settings, CMSIS layer, and HC32 DDL are still being packaged. The repository will ultimately include the known-good Keil project as well as the source; requiring every user to regenerate it from XHCode would reduce reproducibility.

### Is XHCode required?

No. It is useful for generating or inspecting HC32 peripheral configuration, especially during a new board port, but a prepared Keil project should be the normal build path once it is released.

### Is a dedicated XHSC programmer required?

No for the documented development workflow. A correctly connected J-Link can debug and program the HC32F448 over SWD after the HC32 device support is registered. A vendor programmer may still be useful for production fixtures or recovery workflows.

### Are J-Link, XCOM, CAN utilities, and vendor drivers stored in this repository?

Only small files that are redistributable and necessary for HC32 J-Link support are included. Proprietary installers, general-purpose serial terminals, vendor USB drivers, and large third-party tools should be downloaded from their official publishers. This avoids stale binaries, unclear redistribution rights, and unnecessary repository growth.

### Is the second encoder already supported by firmware?

Not in the current public snapshot. The board is shown in the overview, and its hardware files, calibration, communication interface, and dual-encoder control integration are planned releases.

### Is the stored multi-turn position truly absolute?

No. The current implementation accumulates turns during powered operation and persists position to flash. Motion while the controller is unpowered cannot be observed by the motor-side single-turn encoder. An output-side absolute encoder is the intended solution for robust joint position recovery.

### Can another motor, reducer, or power board be used?

Yes, but it is a port rather than a drop-in substitution. Revalidate current-sense scaling, PWM and gate timing, phase order, encoder direction and offset, pole pairs, torque constant, gear ratio, limits, protection, and controller gains before loaded operation.

### Where are the performance numbers?

Quantitative position, torque, bandwidth, thermal, efficiency, and durability results have not yet been released. They will be published with raw data, test conditions, and reproducible analysis under [`results/experiments/`](results/experiments/).

### Why does the repository use Git LFS?

Mechanical models, PCB fabrication outputs, photographs, and experiment datasets can be large. Git LFS keeps repository history practical while preserving versioned release artifacts. Run `git lfs pull` after cloning.

## Citation

OpenEAI-FOC is part of the broader OpenEAI hardware-software effort. If this repository contributes to published work, please cite the OpenEAI-Platform paper and identify the repository version or commit used:

```bibtex
@inproceedings{openeai_platform,
  title  = {OpenEAI-Platform: Open-source Embodied Artificial Intelligence Hardware-Software Unified Platform},
  author = {Jinyuan Zhang and Luoyi Fan and Leiyu Wang and Yeqiang Wang and Yichen Zhu and Cewu Lu and Nanyang Ye},
  year   = {2026}
}
```

A repository-specific citation file and archival DOI will be added when the first stable OpenEAI-FOC release is prepared.

## Contributing

Issues and pull requests are welcome. Read [`CONTRIBUTING.md`](CONTRIBUTING.md) first and include the hardware revision, motor and encoder, supply conditions, and reproducible steps. Changes to the power stage or control parameters should state the tested operating envelope and safety limits.

## License

An open-source license has not yet been selected for this repository. Until a `LICENSE` file is added, the contents remain copyright-protected; public visibility alone does not grant permission to copy, modify, or redistribute them. Firmware, electronics, mechanical designs, documentation, and third-party components may require separate license notices.

## Project Links

- OpenEAI-FOC: <https://github.com/ZJYSII/OpenEAI-FOC>
- OpenEAI-Arm: <https://github.com/eai-yeslab/OpenEAI-Arm>
- OpenEAI-VLA: <https://github.com/eai-yeslab/OpenEAI-VLA>
- Questions and suggestions: use this repository's GitHub Issues
