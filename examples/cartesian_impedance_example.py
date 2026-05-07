"""Hold target pose with adjustable stiffness; print external wrench."""

import argparse
import time

import numpy as np

import franka_rt as fr


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--duration", type=float, default=20.0)
    args = p.parse_args()

    with fr.Robot(args.robot_ip) as robot:
        with robot.start_cartesian_impedance_control() as cic:
            cic.set_stiffness(np.array([300, 300, 300, 30, 30, 30], dtype=float))
            cic.set_damping(np.array([35, 35, 35, 11, 11, 11], dtype=float))

            time.sleep(0.1)
            T = cic.get_state()["O_T_EE"].copy()
            cic.set_target_pose(T)
            print(f"Holding pose. Push the EE — robot will yield and return.")

            t0 = time.monotonic()
            while time.monotonic() - t0 < args.duration:
                s = cic.get_state()
                f = s["O_F_ext_hat_K"]
                print(f"\rExt wrench (xyz): {f[0]:+.2f} {f[1]:+.2f} {f[2]:+.2f} N",
                      end="", flush=True)
                time.sleep(0.1)
            print()


if __name__ == "__main__":
    main()
