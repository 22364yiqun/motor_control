import argparse
import struct
import time

import serial


DM_U2CAN_FRAME = bytearray(
    [
        0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00,
        0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]
)


def int16_range(value, name):
    if value < -32768 or value > 32767:
        raise ValueError(f"{name} out of int16 range: {value}")
    return value


def build_mit_data(pos_deg, vel_deg_s, tau_ff_mnm):
    pos_x100 = int16_range(int(round(pos_deg * 100.0)), "pos_deg_x100")
    vel_x10 = int16_range(int(round(vel_deg_s * 10.0)), "vel_deg_s_x10")
    tau = int16_range(int(round(tau_ff_mnm)), "tau_ff_mNm")
    return struct.pack("<hhhBB", pos_x100, vel_x10, tau, 0, 0)


def build_dm_u2can_frame(can_id, data, tx_status=False):
    if can_id < 0 or can_id > 0x7FF:
        raise ValueError("standard CAN ID must be 0..0x7FF")
    if len(data) != 8:
        raise ValueError("CAN data must be exactly 8 bytes")

    frame = bytearray(DM_U2CAN_FRAME)
    frame[3] = 0x01 if tx_status else 0x03
    frame[13] = can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[21:29] = data
    return frame


def send_frame(ser, can_id, data, verbose=False, tx_status=False):
    frame = build_dm_u2can_frame(can_id, data, tx_status)
    ser.write(frame)
    ser.flush()
    if verbose:
        print(f"raw={frame.hex(' ').upper()}")
    print(f"id=0x{can_id:03X} data={data.hex(' ').upper()}")


def send_output_target_deg(ser, pos_deg, vel_deg_s=0.0, tau_ff_mnm=0.0,
                           can_id=0x201, verbose=False, tx_status=False):
    """Send one output-shaft target command to the motor controller.

    pos_deg is the reducer output-shaft target angle in degrees.
    vel_deg_s is the reducer output-shaft target velocity in deg/s.
    tau_ff_mnm is the output-side feed-forward torque in mN*m.
    """
    data = build_mit_data(pos_deg, vel_deg_s, tau_ff_mnm)
    send_frame(ser, can_id, data, verbose, tx_status)
    return data


def open_can_serial(port="COM5", baudrate=921600, timeout=0.05):
    return serial.Serial(port, baudrate, timeout=timeout)


def extract_dm_u2can_rx_frames(buffer):
    frames = []
    frame_len = 16
    idx = 0
    keep_from = 0

    while idx <= len(buffer) - frame_len:
        if buffer[idx] == 0xAA and buffer[idx + frame_len - 1] == 0x55:
            frame = buffer[idx:idx + frame_len]
            cmd = frame[1]
            info = frame[2]
            can_id = frame[3] | (frame[4] << 8) | (frame[5] << 16) | (frame[6] << 24)
            data = bytes(frame[7:15])
            frames.append((cmd, info, can_id, data, bytes(frame)))
            idx += frame_len
            keep_from = idx
        else:
            idx += 1

    return frames, buffer[keep_from:]


def print_rx_frame(cmd, info, can_id, data, raw=None, verbose=False):
    if verbose and raw is not None:
        print(f"rx_raw={raw.hex(' ').upper()}")
    print(f"rx cmd=0x{cmd:02X} info=0x{info:02X} id=0x{can_id:03X} data={data.hex(' ').upper()}")


def wait_for_ack(ser, ack_id, timeout_s, verbose=False):
    deadline = time.monotonic() + timeout_s
    rx_buffer = bytearray()

    while time.monotonic() < deadline:
        chunk = ser.read(128)
        if chunk:
            rx_buffer.extend(chunk)
            frames, rx_buffer = extract_dm_u2can_rx_frames(rx_buffer)
            for cmd, info, can_id, data, raw in frames:
                if verbose or cmd in (0x02, 0x12, 0xEE):
                    print_rx_frame(cmd, info, can_id, data, raw, verbose)
                if can_id == ack_id:
                    print(f"ack id=0x{can_id:03X} data={data.hex(' ').upper()}")
                    return True
        else:
            time.sleep(0.005)

    print(f"no ack id=0x{ack_id:03X}")
    return False


def listen_frames(ser, duration_s, verbose=False):
    deadline = None if duration_s <= 0 else time.monotonic() + duration_s
    rx_buffer = bytearray()
    print("listening, press Ctrl+C to stop")

    try:
        while deadline is None or time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                rx_buffer.extend(chunk)
                frames, rx_buffer = extract_dm_u2can_rx_frames(rx_buffer)
                for cmd, info, can_id, data, raw in frames:
                    print_rx_frame(cmd, info, can_id, data, raw, verbose)
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nstopped")


def listen_raw(ser, duration_s):
    deadline = None if duration_s <= 0 else time.monotonic() + duration_s
    print("raw listening, press Ctrl+C to stop")

    try:
        while deadline is None or time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                print(chunk.hex(" ").upper())
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nstopped")


def adapter_ping(ser, verbose=False):
    frame = bytearray(DM_U2CAN_FRAME)
    frame[3] = 0x02
    ser.write(frame)
    ser.flush()
    if verbose:
        print(f"ping_raw={frame.hex(' ').upper()}")

    deadline = time.monotonic() + 0.5
    rx = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            rx.extend(chunk)
        else:
            time.sleep(0.005)

    if rx:
        print(f"adapter rx raw={rx.hex(' ').upper()}")
        frames, _ = extract_dm_u2can_rx_frames(rx)
        for cmd, info, can_id, data, raw in frames:
            print_rx_frame(cmd, info, can_id, data, raw, verbose)
    else:
        print("adapter no raw response")


def main():
    parser = argparse.ArgumentParser(
        description="Send custom MIT command through Damiao USB-CAN adapter."
    )
    parser.add_argument("--port", default="COM5", help="serial port, default COM5")
    parser.add_argument(
        "--serial-baud",
        type=int,
        default=921600,
        help="Damiao USB-CAN serial baudrate, default 921600",
    )
    parser.add_argument("--id", type=lambda x: int(x, 0), default=0x201, help="CAN standard ID")
    parser.add_argument("--pos", type=float, default=35.0, help="target position in deg")
    parser.add_argument("--vel", type=float, default=0.0, help="target velocity in deg/s")
    parser.add_argument("--tau", type=float, default=0.0, help="feed-forward torque in mNm")
    parser.add_argument("--interactive", action="store_true", help="input positions repeatedly")
    parser.add_argument("--repeat", action="store_true", help="send the same command continuously")
    parser.add_argument("--listen", action="store_true", help="listen received CAN frames and do not send")
    parser.add_argument("--raw-listen", action="store_true", help="print raw serial bytes and do not parse")
    parser.add_argument("--adapter-ping", action="store_true", help="send Damiao adapter handshake command")
    parser.add_argument("--listen-seconds", type=float, default=0.0, help="listen duration, 0 means forever")
    parser.add_argument("--hz", type=float, default=20.0, help="repeat send rate, default 20 Hz")
    parser.add_argument("--tx-status", action="store_true", help="ask Damiao adapter to return send status")
    parser.add_argument("--ack-id", type=lambda x: int(x, 0), default=0x202, help="CAN ACK standard ID")
    parser.add_argument("--wait-ack", action="store_true", help="wait ACK also in repeat mode")
    parser.add_argument("--ack-timeout", type=float, default=0.5, help="ACK timeout for one command")
    parser.add_argument("--verbose", action="store_true", help="print raw USB-CAN serial frame")
    args = parser.parse_args()

    with serial.Serial(args.port, args.serial_baud, timeout=0.05) as ser:
        if args.raw_listen:
            listen_raw(ser, args.listen_seconds)
        elif args.adapter_ping:
            adapter_ping(ser, args.verbose)
        elif args.listen:
            listen_frames(ser, args.listen_seconds, args.verbose)
        elif args.interactive:
            print("Input target position deg, or q to quit.")
            while True:
                text = input("pos> ").strip()
                if text.lower() in ("q", "quit", "exit"):
                    break
                if not text:
                    continue
                data = build_mit_data(float(text), args.vel, args.tau)
                send_frame(ser, args.id, data, args.verbose, args.tx_status)
                wait_for_ack(ser, args.ack_id, args.ack_timeout, args.verbose)
        elif args.repeat:
            if args.hz <= 0.0:
                raise ValueError("--hz must be greater than 0")
            data = build_mit_data(args.pos, args.vel, args.tau)
            period_s = 1.0 / args.hz
            print(f"repeat {args.hz:g} Hz, press Ctrl+C to stop")
            try:
                while True:
                    send_frame(ser, args.id, data, args.verbose, args.tx_status)
                    if args.wait_ack:
                        wait_for_ack(ser, args.ack_id, args.ack_timeout, args.verbose)
                    time.sleep(period_s)
            except KeyboardInterrupt:
                print("\nstopped")
        else:
            data = build_mit_data(args.pos, args.vel, args.tau)
            send_frame(ser, args.id, data, args.verbose, args.tx_status)
            wait_for_ack(ser, args.ack_id, args.ack_timeout, args.verbose)


if __name__ == "__main__":
    main()
