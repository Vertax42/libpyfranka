"""Hold current configuration with a Python-side joint impedance law.

Demonstrates the JointTorqueControl interface: Python computes torques each
cycle (here a simple PD on q_error) and streams them; libfranka adds gravity
compensation internally.

The Python loop runs at ~100 Hz; the C++ RT thread holds the latest torque
between updates."""

import argparse
import time

import numpy as np

import franka_rt as fr


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--kp", type=float, default=20.0)
    p.add_argument("--kd", type=float, default=2.0)
    args = p.parse_args()

    with fr.Robot(args.robot_ip) as robot:
        with robot.start_joint_torque_control() as jtc:
            time.sleep(0.1)
            q_target = jtc.get_state()["q"].copy()
            print(f"Holding q={q_target} with Kp={args.kp}, Kd={args.kd}")

            t0 = time.monotonic()
            while time.monotonic() - t0 < args.duration:
                s = jtc.get_state()
                tau = args.kp * (q_target - s["q"]) - args.kd * s["dq"]
                jtc.set_target_torques(tau)
                time.sleep(0.01)


if __name__ == "__main__":
    main()
