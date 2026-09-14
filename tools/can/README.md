# CAN Debugging

The current firmware uses classic CAN at the timing described in `source/motor/config.h`. The command and response IDs are `0x201` and `0x202`; see [`docs/protocol/can.md`](../../docs/protocol/can.md).

## Supported Project Tool

[`can_trans.py`](../../can_trans.py) is the repository's reference control utility. It creates the project's custom 8-byte MIT payload and sends it through the serial protocol used by the Damiao USB-CAN adapter:

```bash
python -m pip install -r requirements.txt
python can_trans.py --port COM5 --serial-baud 921600 --pos 10 --vel 0 --tau 0
```

This is preferable to redistributing a vendor GUI because its behavior is reviewable and versioned with the firmware.

## Optional Vendor GUI

The Damiao USB-to-CAN GUI can be useful for manually sending and observing raw CAN frames. Download it from the vendor's maintained [motor-debugging-tool releases](https://github.com/dmBots/motor-debugging-tool) or follow the [USB-to-CAN guide](https://damiao.enactic.ai/en/assets/files/usb-can-conversion-software-c9b6c6574df380e2c249be0e5a307da6.pdf).

The Damiao GUI is proprietary software and is therefore not copied into this repository. Also note that the standard Damiao motor-control GUI targets Damiao motor firmware; it does not automatically understand this repository's custom HC32F448 `0x201` payload. Use its raw-frame function or `can_trans.py`.

## Hardware Checklist

- Set the adapter and controller to the same CAN bitrate.
- Fit 120 Ω termination at both physical ends of the bus.
- Connect CAN_H, CAN_L, and signal ground.
- Verify transceiver standby/enable state.
- Begin with the motor unloaded and the supply current limited.
