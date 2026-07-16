import time

from can_trans import open_can_serial, send_output_target_deg


PORT = "COM5"
SERIAL_BAUD = 115200

def main():
    test_angles = [0.0, 30.0, 60.0, 90.0, 60.0, 30.0, 0.0]

    with open_can_serial(PORT, SERIAL_BAUD) as can:
        for angle in test_angles:
            send_output_target_deg(can, angle, vel_deg_s=0.0, tau_ff_mnm=0.0)
            time.sleep(2.0)


if __name__ == "__main__":
    main()
