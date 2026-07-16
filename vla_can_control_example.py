import time

from can_trans import open_can_serial, send_output_target_deg


def vla_get_target_angle_deg():
    # Replace this with the actual VLA output.
    return 90.0


def main():
    with open_can_serial("COM5") as can:
        while True:
            target_deg = vla_get_target_angle_deg()
            send_output_target_deg(can, target_deg, vel_deg_s=0.0, tau_ff_mnm=0.0)
            time.sleep(0.05)


if __name__ == "__main__":
    main()
