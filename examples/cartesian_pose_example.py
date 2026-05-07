"""Sine-sweep Y-axis at 0.2 Hz, ±5 cm, streaming 4×4 poses at 100 Hz."""

import argparse
import time

import numpy as np

import franka_rt as fr


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--duration", type=float, default=10.0)
    args = p.parse_args()

    with fr.Robot(args.robot_ip) as robot:
        with robot.start_cartesian_pose_control() as cc:
            time.sleep(0.1)
            T0 = cc.get_state()["O_T_EE"].copy()  # (4, 4) numpy

            t0 = time.monotonic()
            while True:
                t = time.monotonic() - t0
                if t > args.duration:
                    break
                T = T0.copy()
                T[1, 3] += 0.05 * np.sin(2 * np.pi * 0.2 * t)
                cc.set_target_pose(T)
                time.sleep(0.01)

            cc.move_to_pose(T0, duration_sec=2.0)
            while cc.is_moving():
                time.sleep(0.05)


if __name__ == "__main__":
    main()
