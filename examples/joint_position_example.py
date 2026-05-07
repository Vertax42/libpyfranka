"""Sine-sweep joint 4 at 0.3 Hz over 5 s, streaming targets at 100 Hz from Python."""

import argparse
import time

import numpy as np

import franka_rt as fr


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument("--freq-hz", type=float, default=0.3)
    p.add_argument("--amp-rad", type=float, default=0.05)
    args = p.parse_args()

    with fr.Robot(args.robot_ip) as robot:
        robot.set_collision_behavior(
            np.full(7, 30.0), np.full(7, 30.0),
            np.full(6, 30.0), np.full(6, 30.0),
        )

        with robot.start_joint_position_control() as jc:
            time.sleep(0.1)  # let the RT thread receive a state
            q0 = jc.get_state()["q"].copy()
            print(f"Starting from q={q0}")

            t0 = time.monotonic()
            while True:
                t = time.monotonic() - t0
                if t > args.duration:
                    break
                target = q0.copy()
                target[3] += args.amp_rad * np.sin(2 * np.pi * args.freq_hz * t)
                jc.set_target_joints(target)
                time.sleep(0.01)  # 100 Hz Python loop

            # Smooth return to start
            jc.move_to_joints(q0, duration_sec=2.0)
            while jc.is_moving():
                time.sleep(0.05)

            s = jc.get_state()
            print(f"Final success rate: {s['control_command_success_rate']:.3f}")


if __name__ == "__main__":
    main()
