# HC32F448 J-Link Device Support

This directory contains the small device-support files needed to add HC32F448 internal-flash programming to a recent SEGGER J-Link installation:

- `JLinkDevices.xml`: device definitions for all 128 KiB and 256 KiB HC32F448 ordering codes in HDSC's CMSIS Pack.
- `HC32F448_128K.FLM`: CMSIS Flash Algorithm for 128 KiB variants.
- `HC32F448_256K.FLM`: CMSIS Flash Algorithm for 256 KiB variants.
- `LICENSE-XHSC.txt`: upstream BSD-3-Clause terms reproduced for binary redistribution.

The `.FLM` files are unmodified copies from HDSC's official [`HDSC.HC32F448.1.0.1.pack`](https://github.com/hdscmcu/pack/blob/master/HDSC.HC32F448.1.0.1.pack). They are not USB drivers and do not replace the [SEGGER J-Link Software](https://www.segger.com/downloads/jlink/).

## Installation

Copy this entire `hc32f448` directory into:

```text
Windows: %APPDATA%\SEGGER\JLinkDevices\XHSC\hc32f448\
Linux:   ~/.config/SEGGER/JLinkDevices/XHSC/hc32f448/
```

Restart every program using the J-Link DLL. Select the exact MCU ordering code printed on the chip. Use SWD at 1 MHz for initial connection. The current firmware expects a 256 KiB device because it reserves flash sector 31.

Do not use an `ATI` definition for a `CTI/CUI` device or the reverse. Do not use an OTP algorithm for ordinary firmware programming; no OTP loader is distributed here.

## Integrity

| File | Size | SHA-256 |
| --- | ---: | --- |
| `HC32F448_128K.FLM` | 32,976 bytes | `1c2a7eb7641c21e108da6ea5e1ffeeea6bffc99fb7ffb2745a2c5cb5085d1cf0` |
| `HC32F448_256K.FLM` | 32,972 bytes | `b1825a11b932f0a71da95b880306eeacc4b4efb993255582038b559013191066` |

The XML values come from the same official Pack: flash base `0x00000000`, work RAM `0x1FFF8000`, and 16 KiB work-RAM size. See [`docs/development-setup.md`](../../../../docs/development-setup.md) for the full J-Link and Keil procedure.
