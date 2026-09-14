# Development Hardware and Software

This page lists the equipment and vendor software needed to build, flash, debug, and test the HC32F448 motor controller. The authoritative vendor download page is the [HC32F448 product page](https://www.xhsc.com.cn/product/1213.html).

## Required Hardware

| Item | Purpose | Required? |
| --- | --- | --- |
| HC32F448-based controller board | Runs the motor-control firmware | Yes |
| Supported three-phase motor | Controlled plant | Yes |
| MA732 encoder, correctly polarized magnet, and mechanical mount | Motor-side electrical/mechanical angle | Yes for the current firmware |
| Current-limited DC bench supply | Safe bring-up and fault-current limiting | Yes for development |
| SWD probe | Flashing and source-level debugging | Yes |
| USB-to-UART adapter | Commands and runtime telemetry | Strongly recommended |
| CAN adapter and two 120 Ω terminations | CAN command and protocol testing | Required only when using CAN |
| Multimeter | Power-rail, continuity, and static checks | Yes |
| Oscilloscope with suitable probes | PWM, gate drive, current sensing, SPI, and CAN validation | Strongly recommended |
| Emergency stop or immediately accessible power disconnect | Safe motor testing | Yes |
| Logic analyzer | Digital-interface diagnosis | Optional |
| Second output-side encoder | Output-position verification and future absolute-position recovery | Planned; not used by the current firmware |

The final hardware release will provide exact part numbers, connector pinouts, supply limits, magnet requirements, and a complete BOM.

## Required Vendor Packages

Use packages intended specifically for **HC32F448**, not similarly named HC32F4A0, HC32A448, or HC32F460 devices.

| Package | Version currently listed by XHSC | Why it is needed | Repository policy |
| --- | --- | --- | --- |
| HC32F448 DDL | `HC32F448_DDL_Rev1.3.0.zip` | CMSIS/device files, startup support, and low-level peripheral drivers | Pin the version; vendor only the required source subset after preserving its license |
| HC32F448 template | `HC32F448_Template_Rev1.1.0.zip` | Reference project layout, startup files, linker configuration, and examples | Download from XHSC; do not commit the archive |
| HC32F448 IDE support | `HC32F448_IDE_Rev1.1.0.zip` | Device description and flash-programming support for supported IDEs | Install locally; do not commit installers |
| XHCode | `XHSC XHCode V1.10.3 Setup.zip` | Optional graphical configuration/code-generation assistant | Optional; install locally; do not commit |

The versions above are those shown on the vendor page when this document was written. Check the official page for newer releases and read the matching release notes and errata before changing versions.

## DDL Modules Used by This Firmware

The current `source/inc/hc32f4xx_conf.h` enables the low-level modules. The application directly requires:

```text
ICG, UTILITY,
ADC, AOS, CLK, DMA, EFM, FCG, GPIO, INTERRUPTS,
MCAN, PWC, SPI, TMR4, USART
```

`CMP` and `SRAM` are currently enabled in the configuration but are not called directly by the application. They may be disabled only after a complete build and hardware regression test.

The MA732 is read directly through SPI; no separate MA732 software library is required. FOC, SVPWM, and MIT control are implemented in this repository.

## Compiler and IDE

Choose one supported toolchain and document its exact version in test reports:

- Keil MDK-ARM with the HC32F448 IDE/device support installed;
- IAR Embedded Workbench with the corresponding XHSC patch/support package; or
- Arm GNU Toolchain with a maintained CMake/Make project and the correct startup/linker files.

XHCode is an optional configuration assistant. It is not a compiler, debugger, or replacement for the DDL/device support package.

## Flashing and Debugging

### Official path: XH-Link

XH-Link is the vendor-supported debug path and is the safest baseline for a reproducible setup. Install the software specified by the current XHSC documentation and connect SWDIO, SWCLK, GND, target reference voltage, and preferably NRST.

### J-Link alternative

A genuine SEGGER J-Link can be used as an SWD probe when the selected IDE/device pack supplies working HC32F448 device and flash-algorithm support. However, SEGGER's public supported-device list does not currently identify HC32F448 explicitly. Do not promise plug-and-play standalone J-Link flashing until the exact probe model, J-Link software version, IDE, device selection, flash algorithm, and board revision have been tested together.

For this reason, the project should document J-Link as a **tested alternative**, not as the only guaranteed programmer. After validation, record the working configuration and a short flash/debug procedure here.

### XHSC programming utilities

The vendor page also provides XHSC ISP, Programmer Config Tool, and XH-Link-Writer. They are not required for normal source-level development when SWD flashing already works. Keep them as optional tools for UART/bootloader recovery, option-byte or programming configuration, XH-Link-based writing, or production workflows.

If only J-Link is used, XH-Link-specific USB drivers are unnecessary. The J-Link software and driver are still required, as is a valid HC32F448 flash algorithm or device-support integration.

## What Belongs in This Repository?

Do **not** commit vendor installers, downloaded ZIP archives, IDE executables, J-Link software, XHCode, XHSC programming utilities, or copies of vendor manuals. They are large, become stale, and may have redistribution restrictions. Link to the official HC32F448 page instead.

For reproducible builds, the DDL is different: its source files are compile-time dependencies. The preferred release policy is:

1. Pin an exact DDL release, initially `HC32F448_DDL_Rev1.3.0`.
2. Add only the device, CMSIS/startup, system, and LL driver files required by this firmware under `third_party/xhsc/`.
3. Preserve the upstream directory structure, copyright headers, BSD-3-Clause license text, version, and source URL.
4. Add `third_party/xhsc/README.md` and `LICENSE`/`LICENSE.txt` from the upstream package.
5. Do not modify vendor files silently; keep project-specific configuration outside the vendored tree where practical.
6. Record the vendor SHA-512 checksum published alongside the DDL archive.

Until that audited subset is added, users must download the DDL and template from the official product page. A link alone is acceptable for installers and manuals, but it is not ideal for a source dependency needed to reproduce the build.

## Minimum Recommended Development Setup

For a new contributor, the practical minimum is:

```text
HC32F448 controller + motor + MA732
current-limited bench supply + multimeter + oscilloscope + emergency stop
XH-Link (baseline) or a project-verified J-Link setup
USB-UART adapter
CAN adapter and termination when testing CAN
HC32F448 DDL + template + IDE support
Keil/IAR/GCC toolchain
XHCode only if graphical configuration assistance is desired
```
