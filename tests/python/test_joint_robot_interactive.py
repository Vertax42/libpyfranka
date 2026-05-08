#!/usr/bin/env python3
"""On-robot interactive smoke test for franka_rt.JointPositionControl.

SAFETY MODEL
============
- Operator MUST keep e-stop within reach.
- Every motion phase prompts for an explicit Enter before moving.
- All motions are RELATIVE to the robot's current joint position at script
  start.  Nothing is hardcoded to absolute joint angles.
- Default test joint is joint 3 (elbow) — its motion is the most predictable
  to watch and the least likely to take the EE somewhere awkward.  Override
  with `--joint <0..6>`.
- Maximum streaming amplitude defaults to 0.05 rad (≈ 3°) — well below any
  workspace concern.  Override at your own risk with `--sine-amp`.
- Robot is left where it is on exit.
- Ctrl+C at any time stops streaming and exits.

PHASES
======
  1. Connect + read state          (no motion)
  2. Stream-hold current q         (no commanded motion, control loop up)
  3. move_to_joints +0.05 rad on chosen joint (auto duration, ≈3° move)
  4. 30-Hz sine on chosen joint, 30 seconds
  5. 30-Hz sine on chosen joint, 5 minutes  (sustained reflex test; opt-in)

USAGE
=====
    cd ~/libpyfranka
    python tests/python/test_joint_robot_interactive.py --ip 192.168.99.111
    # --include-long      run phase 5 (5-min run)
    # --skip-stream-hold  skip phase 2
    # --joint <i>         use joint i (default 3 = elbow)
    # --sine-amp <rad>    streaming amplitude (default 0.05)
    # --sine-freq <hz>    streaming frequency (default 0.1)
    # --phase <n>         run only phase n

PRE-FLIGHT CHECKLIST
====================
  [ ] FCI is activated on Franka Desk (light on robot is white)
  [ ] Robot is unlocked, not in user-stop
  [ ] User-stop button physically in operator's hand
  [ ] No people, tools, or fragile items within 50 cm of EE
  [ ] Robot is in a comfortable middle-of-workspace pose (NOT at joint limits)
  [ ] Chosen joint has at least ±0.1 rad of headroom in both directions
  [ ] Network: ping <ROBOT-IP> succeeds, < 1 ms RTT typical
"""

import argparse
import math
import sys
import time
import traceback

import numpy as np

import franka_rt as fr


# ---------------------------------------------------------------------------
# Pretty-print helpers
# ---------------------------------------------------------------------------

def vfmt(v, w=7, p=4):
    return "[" + ", ".join(f"{x:+{w}.{p}f}" for x in v) + "]"


def banner(title):
    line = "=" * 78
    print()
    print(line)
    print(f"  {title}")
    print(line)


def confirm(msg):
    print()
    print(f"[CONFIRM] {msg}")
    print("[CONFIRM] Operator: keep e-stop in hand.")
    try:
        input("[CONFIRM] Press Enter to continue (Ctrl+C aborts) ... ")
    except KeyboardInterrupt:
        print("\n*** ABORTED by operator before motion ***")
        sys.exit(1)


def print_state_brief(state, prefix="  "):
    """Format a state dict from Robot.read_once or JointPositionControl.get_state."""
    print(f"{prefix}q [rad]      : {vfmt(state['q'])}")
    print(f"{prefix}q [deg]      : "
          + ", ".join(f"{math.degrees(v):+7.2f}" for v in state["q"]))
    print(f"{prefix}dq [rad/s]   : {vfmt(state['dq'])}")
    if "tau_J" in state:
        print(f"{prefix}tau_J [Nm]   : {vfmt(state['tau_J'], w=7, p=2)}")
    if "tau_ext_hat_filtered" in state:
        print(f"{prefix}tau_ext      : {vfmt(state['tau_ext_hat_filtered'], w=7, p=2)} Nm")
    if "has_error" in state:
        print(f"{prefix}has_error    : {state['has_error']}")
    if "current_errors" in state:
        ce = state["current_errors"]
        if isinstance(ce, str):
            print(f"{prefix}errors_str   : '{ce}'")
        else:
            print(f"{prefix}errors_pack  : {ce:#x}")
    if "robot_mode" in state:
        print(f"{prefix}robot_mode   : {state['robot_mode']}")
    if "control_command_success_rate" in state:
        print(f"{prefix}cmd success%  : "
              f"{100.0*state['control_command_success_rate']:.1f}")


def assert_no_error(state, where):
    if isinstance(state.get("has_error", False), bool) and state.get("has_error"):
        ce = state.get("current_errors", "?")
        raise RuntimeError(
            f"[{where}] Robot reports errors: {ce}.  "
            "Run robot.automatic_error_recovery() and/or recover on Franka Desk."
        )
    if (isinstance(state.get("current_errors", 0), int)
            and state.get("current_errors", 0) != 0):
        raise RuntimeError(
            f"[{where}] Robot has packed errors "
            f"(uint64=0x{state['current_errors']:x}).  Recover."
        )


# ---------------------------------------------------------------------------
# Phase implementations
# ---------------------------------------------------------------------------

def phase1_connect(args):
    banner("PHASE 1 — Connect to Franka, read one state.  No motion.")
    confirm("About to call fr.Robot(...).  Robot will not move.")

    robot = fr.Robot(args.ip, fr.RealtimeConfig.kIgnore)
    print("\n  Connected.  Reading state...")
    state = robot.read_once()

    # Auto-recover stale errors / Reflex from a previous failed run.
    if state.get("has_error") or state.get("current_errors", 0):
        print(f"  Detected stale error: {state.get('current_errors')}")
        confirm("Run automatic_error_recovery() to clear it?")
        robot.automatic_error_recovery()
        time.sleep(0.2)
        state = robot.read_once()

    if state.get("robot_mode") == fr.RobotMode.UserStopped:
        raise RuntimeError(
            "Robot is in UserStopped mode — release the user-stop "
            "button on the pendant and re-run."
        )

    print_state_brief(state)
    assert_no_error(state, "phase1.read_once")
    print("\n[PHASE 1 PASS]  connected, no errors.")
    return robot, state


def phase2_stream_hold(robot, ref_state, args):
    """Start the joint controller, hold current pose for `--hold-sec`.

    The OTG should hold rock-solid at q0 with stream_dq=stream_ddq=0.  Any
    error here means start-up is wrong (initial seeding, warmup gate, ...).
    """
    banner(f"PHASE 2 — Stream-hold current q for {args.hold_sec:.1f} s.")
    print("  No commanded motion.  Operator can gently push the arm to feel")
    print("  the joint impedance — the arm should resist and return.")
    confirm("About to enter joint-impedance hold.")

    q0 = ref_state["q"]
    print(f"\n  q0 [rad]: {vfmt(q0)}")

    with robot.start_joint_position_control() as ac:
        # Give the controller's warmup a moment, then start sending the
        # same q every loop so the OTG has a steady setpoint.
        t0 = time.time()
        last_print = 0.0
        while time.time() - t0 < args.hold_sec:
            ac.set_target_joints(q0)
            now = time.time()
            if now - last_print > 2.0:
                s = ac.get_state()
                err = s.get("has_error", False)
                sr = s.get("control_command_success_rate", 0.0)
                dq_max = max(abs(v) for v in s["dq"])
                print(f"  [hold] t={now-t0:5.1f}/{args.hold_sec:.0f}  "
                      f"|dq|max={dq_max:.4f}rad/s  "
                      f"cmd_succ={100*sr:.1f}%  err={err}")
                assert_no_error(s, "phase2.hold")
                last_print = now
            time.sleep(0.033)

        # Final state read-back
        final = ac.get_state()
        assert_no_error(final, "phase2.final")

    print("\n[PHASE 2 PASS]  hold completed without errors.")


def phase3_move_to(robot, ref_state, args):
    """Tiny scripted move on the chosen joint, then return."""
    banner(f"PHASE 3 — move_to_joints, joint {args.joint} +{args.move_delta:+.3f} rad")
    print(f"  Auto-duration (let OTG pick a safe T from |Δq|).")
    confirm("About to issue a non-blocking joint move.")

    fresh = robot.read_once()
    assert_no_error(fresh, "phase3.fresh")
    q0 = list(fresh["q"])
    target = list(q0)
    target[args.joint] = q0[args.joint] + args.move_delta

    print(f"\n  start q[{args.joint}] = {q0[args.joint]:+.4f} rad")
    print(f"  target q[{args.joint}] = {target[args.joint]:+.4f} rad")

    with robot.start_joint_position_control() as ac:
        # Issue the move.  duration_sec=0 → controller auto-computes.
        ac.move_to_joints(target, 0.0)

        # Watch progress until is_moving() returns False.
        t0 = time.time()
        last_print = 0.0
        while ac.is_moving():
            now = time.time()
            if now - last_print > 0.5:
                s = ac.get_state()
                err = s.get("has_error", False)
                sr = s.get("control_command_success_rate", 0.0)
                cur = s["q"][args.joint]
                rem = target[args.joint] - cur
                print(f"  [move] t={now-t0:5.2f}  q[{args.joint}]={cur:+.4f}  "
                      f"to-go={rem:+.4f}  cmd_succ={100*sr:.1f}%  err={err}")
                assert_no_error(s, "phase3.move")
                last_print = now
            time.sleep(0.05)
            # Safety: if the move overruns 30 s something is wrong.
            if time.time() - t0 > 30.0:
                raise RuntimeError("move_to_joints did not complete within 30 s")

        final = ac.get_state()
        err = abs(final["q"][args.joint] - target[args.joint])
        print(f"\n  Move done in {time.time()-t0:.2f} s.  "
              f"final q[{args.joint}] = {final['q'][args.joint]:+.4f}  "
              f"|err| = {err*1000:.2f} mrad")
        assert_no_error(final, "phase3.final")

        # Return the joint to where it started, so subsequent phases see
        # a clean state.
        ac.move_to_joints(q0, 0.0)
        while ac.is_moving():
            time.sleep(0.05)
        assert_no_error(ac.get_state(), "phase3.return")

    print("\n[PHASE 3 PASS]  move + return completed.")


def _run_sine(ac, q0, joint_idx, duration, amp, freq, rate, label):
    """Stream a sinusoid on q[joint_idx], with a quintic ramp-in/out envelope.

    The envelope keeps the very first and very last commands at q0 with
    zero velocity AND acceleration, eliminating the "hard start"/"hard
    stop" discontinuities that would otherwise dominate edge-case errors.
    """
    dt = 1.0 / rate
    t0 = time.time()
    last_print = 0.0
    iters = 0
    min_rate = 1.0

    ramp_sec = 2.0

    def envelope(t_):
        if t_ <= 0:
            return 0.0
        if t_ >= ramp_sec and t_ <= duration - ramp_sec:
            return 1.0
        if t_ >= duration:
            return 0.0
        # Ramp-in for t_ < ramp_sec, ramp-out for t_ > duration - ramp_sec
        if t_ < ramp_sec:
            s = t_ / ramp_sec
        else:
            s = (duration - t_) / ramp_sec
        return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5

    while True:
        loop_t0 = time.time()
        t = loop_t0 - t0
        if t >= duration:
            break

        env = envelope(t)
        q = list(q0)
        q[joint_idx] = q0[joint_idx] + amp * env * math.sin(2 * math.pi * freq * t)

        ac.set_target_joints(q)
        iters += 1

        if loop_t0 - last_print > 2.0:
            s = ac.get_state()
            err = s.get("has_error", False)
            sr = s.get("control_command_success_rate", 0.0)
            min_rate = min(min_rate, sr)
            print(f"  [{label}] t={t:6.1f}/{duration:.0f}  "
                  f"q[{joint_idx}]={s['q'][joint_idx]:+.4f}rad  "
                  f"cmd_succ={100*sr:.1f}%  err={err}")
            assert_no_error(s, f"{label}.streaming")
            last_print = loop_t0

        sleep_for = dt - (time.time() - loop_t0)
        if sleep_for > 0:
            time.sleep(sleep_for)

    print(f"  [{label}] iters={iters}  min cmd_succ={100*min_rate:.1f}%")


def phase4_short_sine(robot, ref_state, args):
    banner(f"PHASE 4 — 30-Hz sinusoid on joint {args.joint}, "
           f"{args.short_sec:.0f} s.")
    print(f"  Stream q[{args.joint}] = q0 + {args.sine_amp*1000:.1f} mrad × "
          f"sin(2π × {args.sine_freq:.2f} Hz × t).")
    print("  Python writer at 30 Hz, RT thread integrates at 1 kHz via OTG.")
    print(f"  Duration: {args.short_sec:.1f} s.   No reflex expected.")
    confirm(f"About to stream sinusoidal motion (max {args.sine_amp*1000:.1f} mrad).")

    fresh = robot.read_once()
    assert_no_error(fresh, "phase4.fresh")
    q0 = list(fresh["q"])

    print("\n  --- Phase 4 start state ---")
    print_state_brief(fresh)

    with robot.start_joint_position_control() as ac:
        _run_sine(ac, q0, args.joint, args.short_sec,
                  args.sine_amp, args.sine_freq, rate=30, label="phase4")

    print(f"\n[PHASE 4 PASS]  {args.short_sec:.0f} s of streaming done.")


def phase5_long_sine(robot, ref_state, args):
    banner(f"PHASE 5 — sustained 30-Hz sinusoid on joint {args.joint}, "
           f"{args.long_sec/60:.1f} min.")
    print("  Same shape as phase 4 but {0:.0f}× longer.  "
          "This is the main reflex test.".format(args.long_sec / args.short_sec))
    confirm(f"About to stream for {args.long_sec/60:.1f} minutes.")

    fresh = robot.read_once()
    assert_no_error(fresh, "phase5.fresh")
    q0 = list(fresh["q"])

    with robot.start_joint_position_control() as ac:
        _run_sine(ac, q0, args.joint, args.long_sec,
                  args.sine_amp, args.sine_freq, rate=30, label="phase5")

    print(f"\n[PHASE 5 PASS]  {args.long_sec/60:.1f} min of streaming done.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default="192.168.99.111")
    p.add_argument("--joint", type=int, default=3,
                   help="Joint index 0..6 to exercise (default 3 = elbow)")
    p.add_argument("--hold-sec", type=float, default=5.0,
                   help="Phase-2 hold duration")
    p.add_argument("--move-delta", type=float, default=0.05,
                   help="Phase-3 single-joint delta in radians (default 0.05)")
    p.add_argument("--sine-amp", type=float, default=0.05,
                   help="Streaming sinusoid amplitude in radians (default 0.05)")
    p.add_argument("--sine-freq", type=float, default=0.1,
                   help="Streaming sinusoid frequency in Hz (default 0.1)")
    p.add_argument("--short-sec", type=float, default=30.0,
                   help="Phase-4 duration")
    p.add_argument("--long-sec", type=float, default=300.0,
                   help="Phase-5 duration (only with --include-long)")
    p.add_argument("--skip-stream-hold", action="store_true",
                   help="Skip phase 2")
    p.add_argument("--include-long", action="store_true",
                   help="Run phase 5 (5-min sustained stream)")
    p.add_argument("--phase", type=int, default=None,
                   help="Run only the given phase (1..5)")
    args = p.parse_args()

    if not (0 <= args.joint <= 6):
        p.error("--joint must be in 0..6")

    print(__doc__)

    try:
        robot, state = phase1_connect(args)

        def should_run(n):
            return args.phase is None or args.phase == n

        if args.phase in (None, 1):
            pass  # phase 1 already done

        if should_run(2) and not args.skip_stream_hold:
            phase2_stream_hold(robot, state, args)

        if should_run(3):
            phase3_move_to(robot, state, args)

        if should_run(4):
            phase4_short_sine(robot, state, args)

        if should_run(5) and args.include_long:
            phase5_long_sine(robot, state, args)

        banner("ALL REQUESTED PHASES PASSED")

    except KeyboardInterrupt:
        print("\n\n*** Ctrl+C — aborting ***")
        sys.exit(130)
    except Exception as e:
        print(f"\n\n*** EXCEPTION CAUGHT ***")
        traceback.print_exc()
        print(f"\n  {type(e).__name__}: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
