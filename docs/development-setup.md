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

A genuine SEGGER J-Link can be used over SWD, but HC32F448 must be added to the J-Link device database when it is absent from the installed SEGGER device list. The missing piece is not a special USB driver from XHSC; it is the HC32F448 CMSIS Flash Algorithm (`.FLM`) plus a J-Link XML device description.

The instructions below follow SEGGER's current [J-Link Device Support Kit](https://kb.segger.com/DSK) mechanism. SEGGER recommends a user-level `JLinkDevices` directory instead of modifying the J-Link installation directory, so the custom device definition survives J-Link upgrades.

#### 1. Install the required packages

Install the current [SEGGER J-Link Software and Documentation Pack](https://www.segger.com/downloads/jlink/) and the HDSC HC32F448 CMSIS Pack/IDE support. The official HDSC Pack repository contains [`HDSC.HC32F448.1.0.1.pack`](https://github.com/hdscmcu/pack) and the device-specific `.FLM` files.

A CMSIS `.pack` file is a ZIP archive. Open or extract it and locate:

```text
FlashARM/HC32F448_128K.FLM
FlashARM/HC32F448_256K.FLM
```

Select the file by the **full MCU part number**, not merely by package size:

| MCU code-flash size | Flash loader | Maximum size |
| --- | --- | ---: |
| 128 KiB variant | `HC32F448_128K.FLM` | `0x20000` |
| 256 KiB variant | `HC32F448_256K.FLM` | `0x40000` |

The current firmware reserves sector 31 of a 256 KiB device, so it appears to expect a 256 KiB HC32F448 variant. Confirm the exact ordering code from the PCB schematic or chip marking before installing the loader.

#### 2. Create the user device directory

For J-Link software V7.62 or later, create a directory such as:

```text
Windows: %APPDATA%\SEGGER\JLinkDevices\XHSC\HC32F448\
Linux:   ~/.config/SEGGER/JLinkDevices/XHSC/HC32F448/
```

Copy the selected `.FLM` file into that directory. Do not copy it into `Program Files/SEGGER`; files in the installation directory may be removed by an update.

#### 3. Add `JLinkDevices.xml`

For a 256 KiB part, create this file next to `HC32F448_256K.FLM`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<Database>
  <Device>
    <ChipInfo Vendor="XHSC"
              Name="HC32F448_CUSTOM_256K"
              Core="JLINK_CORE_CORTEX_M4"
              WorkRAMAddr="0x1FFF8000"
              WorkRAMSize="0x00004000" />
    <FlashBankInfo Name="Internal Code Flash"
                   BaseAddr="0x00000000"
                   AlwaysPresent="1">
      <LoaderInfo Name="HC32F448 256K"
                  Loader="HC32F448_256K.FLM"
                  LoaderType="FLASH_ALGO_TYPE_OPEN"
                  MaxSize="0x00040000" />
    </FlashBankInfo>
  </Device>
</Database>
```

For a 128 KiB device, change the custom device name, loader name, loader filename, and `MaxSize` to `0x00020000`. The flash base address remains `0x00000000`. The work-RAM address and size above come from HDSC's official HC32F448 CMSIS Pack description.

The deliberately distinct `HC32F448_CUSTOM_256K` name avoids accidentally overriding a future SEGGER-provided definition. If the IDE requires the exact HDSC part number, use the confirmed ordering code as the XML `Name` and select that same name in the debugger configuration.

#### 4. Restart and verify J-Link

Close all applications using the J-Link DLL, then reopen J-Link Commander, J-Flash, or the IDE. Select `HC32F448_CUSTOM_256K`, `SWD`, and a conservative initial SWD speed such as 1 MHz.

In J-Link Commander, verify connection and memory access before erasing anything:

```text
JLinkExe -device HC32F448_CUSTOM_256K -if SWD -speed 1000
```

Then issue `connect`, `reset`, and a read-only memory display command. Only after a stable connection should you test erase/program/verify using a known image. Do not program the OTP loader (`HC32F448_otp.FLM`) during ordinary firmware development.

If the custom name does not appear, check that the file is named exactly `JLinkDevices.xml`, that it and the `.FLM` share the documented directory, and that a current J-Link DLL is actually being used by the IDE. SEGGER documents the supported XML search locations in [Using Flashloader with different IDEs](https://kb.segger.com/Using_Flashloader_with_different_IDEs).

#### 5. Configure the IDE

In Keil, install the HDSC HC32F448 CMSIS Pack, select the exact HC32F448 device, choose J-Link/J-TRACE Cortex as the debug adapter, use SWD, and confirm that the matching `HC32F448_128K.FLM` or `HC32F448_256K.FLM` algorithm is selected under the Flash Download settings. The CMSIS Pack is also listed in Arm's [HC32F448 device catalog](https://www.keil.arm.com/family/hdsc-hc32f448-series/).

Record the J-Link probe model, J-Link software/DLL version, IDE version, exact MCU ordering code, XML device name, `.FLM` checksum, SWD speed, and board revision after the procedure is validated on hardware. Until that test is completed, J-Link remains a documented integration path rather than the project's guaranteed baseline programmer.

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
