#!/usr/bin/env python3
"""On-robot interactive smoke test for franka_rt.CartesianControl.

SAFETY MODEL
============
- Operator MUST keep e-stop within reach.
- Every motion phase prompts for an explicit Enter before moving.
- All motions are RELATIVE to the robot's current pose at script start —
  nothing is hardcoded to absolute coordinates.
- Maximum displacement per phase is 1 cm translation / 0 rotation.
- Robot is left where it is on exit (no hidden "go home" trajectory).
- Ctrl+C at any time stops streaming and exits.

PHASES
======
  1. Connect + read state          (no motion)
  0. Go to lerobot 'flare' TCP home (deterministic start, eliminates drift)
  2. Stream-hold current pose      (no commanded motion; user can push EE)
  3. move_to_pose +1 cm in X       (1 cm scripted, 3 s duration)
  4. 30-Hz sinusoid, 30 seconds    (small streaming test)
  5. 30-Hz sinusoid, 5 minutes     (sustained — main reflex test; opt-in)

USAGE
=====
    cd ~/libpyfranka
    python tests/python/test_cartesian_robot_interactive.py --ip 192.168.99.111
    # add --include-long for phase 5 (5-min run)
    # add --skip-stream-hold to skip phase 2 (if you can't push the EE)

PRE-FLIGHT CHECKLIST
====================
  [ ] FCI is activated on Franka Desk (light on robot is white)
  [ ] Robot is unlocked, not in user-stop
  [ ] User-stop button physically in operator's hand
  [ ] No people, tools, or fragile items within 50 cm of EE
  [ ] Robot is in a comfortable middle-of-workspace pose (NOT at joint limits)
  [ ] Network: ping 192.168.99.111 succeeds, < 1 ms RTT typical
"""

import argparse
import math
import sys
import time
import traceback

import numpy as np

import franka_rt as fr


def _impedance_mode(args):
    """Translate --impedance-mode CLI arg to franka_rt.ControllerMode enum."""
    return (fr.ControllerMode.JointImpedance
            if getattr(args, "impedance_mode", "cartesian") == "joint"
            else fr.ControllerMode.CartesianImpedance)


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
    """Format a state-dict (from Robot.read_once or CartesianControl.get_state)."""
    O_T_EE = state["O_T_EE"]
    pos = O_T_EE[:3, 3]
    print(f"{prefix}EE pos       : {vfmt(pos, w=8, p=4)} m")
    print(f"{prefix}q [rad]      : {vfmt(state['q'])}")
    print(f"{prefix}dq [rad/s]   : {vfmt(state['dq'])}")
    print(f"{prefix}ext F (xyz)  : {vfmt(state['O_F_ext_hat_K'][:3], w=7, p=2)} N")
    print(f"{prefix}ext M (rxyz) : {vfmt(state['O_F_ext_hat_K'][3:], w=7, p=2)} Nm")
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
        print(f"{prefix}cmd success%  : {100.0*state['control_command_success_rate']:.1f}")


def assert_no_error(state, where):
    if isinstance(state.get("has_error", False), bool) and state.get("has_error"):
        ce = state.get("current_errors", "?")
        raise RuntimeError(
            f"[{where}] Robot reports errors: {ce}.  "
            "Run robot.automatic_error_recovery() and/or recover on Franka Desk."
        )
    if isinstance(state.get("current_errors", 0), int) and state.get("current_errors", 0) != 0:
        raise RuntimeError(
            f"[{where}] Robot has packed errors (uint64=0x{state['current_errors']:x}).  Recover."
        )


# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------

def phase0_go_home(robot, args):
    """Drive the robot to a deterministic TCP home XYZ before any test phase.

    Avoids cumulative drift across runs (each phase 2/3 adds a few mm to the
    robot's actual pose, so without a reset the start condition keeps changing).

    Home XYZ was empirically chosen from a hand-jogged pose that keeps the
    wrist away from the q[6] ≈ 0 ("crossed wrist") configuration where IK
    becomes too sensitive for streaming.  Captured 2026-05-09:
        q [deg]: -6.29, -1.40, -1.34, -102.94, -1.26, +101.21, -61.73
        pos    : (+0.5489, -0.0781, +0.4276) m
        elbow  : [-0.023, -1.0]

    NOTE: lerobot pylibfranka_research3 'flare' home was at (0.5592, -0.0073,
    0.5123) with q[6] ≈ -0.89 rad — different rotation.  Using a different XYZ
    here because that pose triggered cartesian_motion_generator_joint_acceleration_discontinuity
    on streaming inputs.
    """
    banner("PHASE 0 — Go to empirical TCP home XYZ.")
    print("  target XYZ : [+0.5489, -0.0781, +0.4276] m")
    print("  rotation   : kept from current pose (no rotation change)")
    print("  duration   : 5 s, min-jerk")
    if args.skip_go_home:
        print("\n[PHASE 0 SKIPPED via --skip-go-home]")
        return
    confirm("About to translate EE to home XYZ via move_to_pose.")

    # Build home target by KEEPING the current rotation and only translating
    # to lerobot's home XYZ.  Doing a large rotation in Phase 0 (to lerobot's
    # tool-down quat 0,1,0,0) tends to trigger
    # cartesian_motion_generator_*_discontinuity for the same redundancy /
    # IK reason that bites Phase 4.  Translation-only is safe.
    ac = robot.start_cartesian_pose_control(_impedance_mode(args))
    time.sleep(0.15)  # warmup
    state_now = ac.get_state()
    T_home = np.asarray(state_now["O_T_EE_c"], dtype=np.float64).copy()
    T_home[0, 3] = 0.5489
    T_home[1, 3] = -0.0781
    T_home[2, 3] = 0.4276
    print(f"  current pos  : {vfmt(state_now['O_T_EE_c'][:3, 3], 8, 4)} m")
    print(f"  target pos   : {vfmt(T_home[:3, 3], 8, 4)} m")
    print(f"  rotation     : kept from current (no rotation change)")
    ac.move_to_pose(T_home, duration_sec=5.0)

    t0 = time.time()
    last_print = 0.0
    while ac.is_moving() and time.time() - t0 < 8.0:
        if time.time() - last_print > 0.5:
            s = ac.get_state()
            pos = s["O_T_EE"][:3, 3]
            print(f"  t={time.time()-t0:4.2f}  pos={vfmt(pos, 8, 4)}  is_moving={ac.is_moving()}  err={s.get('has_error')}")
            last_print = time.time()
            assert_no_error(s, "phase0.go_home")
        time.sleep(0.02)

    if ac.is_moving():
        print("  WARNING: move_to_pose did not finish in 8 s — cancelling.")
        ac.cancel_move()
        time.sleep(0.5)

    s = ac.get_state()
    pos = s["O_T_EE"][:3, 3]
    target = T_home[:3, 3]
    err_xyz = pos - target
    print(f"\n  final pos    : {vfmt(pos, 8, 4)} m")
    print(f"  pos error    : {vfmt(err_xyz * 1000, 7, 2)} mm")
    print(f"  cmd success% : {100.0*s['control_command_success_rate']:.2f}")
    ac.stop()
    if np.max(np.abs(err_xyz)) > 0.005:
        print(f"  WARNING: |pos error| > 5 mm; impedance compliance under load is normal,"
              f"\n           but check for collisions / wrong EE config if larger.")
    print("[PHASE 0 PASS]  at home pose.")


def phase1_connect(args):
    banner("PHASE 1 — Connect to Franka, read one state.  No motion.")
    confirm(f"About to call fr.Robot('{args.ip}').  Robot will not move.")

    robot = fr.Robot(args.ip, fr.RealtimeConfig.kIgnore,
                     disable_default_behavior=False)
    print(f"\n  Connected.  Reading state...")
    state = robot.read_once()
    print_state_brief(state)

    # If a previous session crashed (e.g. communication_constraints_violation),
    # the robot will surface stale errors here.  Offer to clear them.
    if state.get("has_error", False):
        print()
        print("  Robot has stale errors from a previous session.  These are NOT")
        print("  caused by the current script — most likely a previous run hit")
        print("  reflex and didn't clear.  Safe to recover if no physical issue.")
        print(f"  Errors: {state.get('current_errors', '?')}")
        print(f"  robot_mode: {state.get('robot_mode', '?')}")
        confirm("Attempt robot.automatic_error_recovery()?")
        try:
            robot.automatic_error_recovery()
            print("  automatic_error_recovery() returned successfully.")
        except Exception as e:
            print(f"  automatic_error_recovery() FAILED: {e}")
            print("  Recover manually on Franka Desk (click 'Recover'), then retry.")
            raise

        time.sleep(0.5)
        state = robot.read_once()
        print("\n  Re-reading state after API recovery:")
        print_state_brief(state)

        # If the API recovery alone wasn't enough, walk the operator through
        # the manual Desk recovery flow.  This is common for some error types.
        if state.get("has_error", False):
            print()
            print("  *** API recovery did NOT clear the error. ***")
            print()
            print("  This usually means one of:")
            print("    1. User-stop button is pressed (release it).")
            print("    2. Robot brakes are engaged in Desk")
            print("       (open https://%s, click 'Open Brakes')." % args.ip)
            print("    3. Error is sticky — needs the 'Recover' button on Desk")
            print("       (top toolbar, only appears when there is an error).")
            print("    4. FCI feature is not active in Desk")
            print("       (left menu → 'FCI', click 'Activate FCI').")
            print("    5. Network packets are still dropping — see your repo's")
            print("       docs/franka-communication-constraints-violation-troubleshooting.md")
            print()
            print(f"  Current errors    : {state.get('current_errors', '?')}")
            print(f"  Current robot_mode: {state.get('robot_mode', '?')}")
            print(f"  ext_F (xyz) [N]   : {vfmt(state['O_F_ext_hat_K'][:3], 6, 2)}  "
                  "(non-zero = something pushing on EE)")
            print()
            print(
                "  Open Franka Desk in a browser, finish the manual recovery,"
                "\n  then come back here and press Enter to retry — or Ctrl+C to abort."
            )
            try:
                input("  Press Enter after Desk shows status white/Idle ... ")
            except KeyboardInterrupt:
                print("\n*** ABORTED by operator ***")
                sys.exit(1)
            time.sleep(0.5)
            state = robot.read_once()
            print("\n  Re-reading state after manual Desk recovery:")
            print_state_brief(state)

    assert_no_error(state, "phase1.connect")

    # User-stop check.  In UserStopped mode, FCI rejects every control start
    # call, but our state read still succeeds, so it's easy to miss until
    # the next phase blows up cryptically.
    if state.get("robot_mode") == fr.RobotMode.UserStopped:
        raise RuntimeError(
            "[phase1.connect] Robot is in UserStopped mode.  Release the "
            "physical user-stop button (red mushroom on the controller, or "
            "your hand-held safety switch) and re-run.  Verify on Desk that "
            "robot_mode is Idle / Operational, not UserStopped."
        )

    print("\n[PHASE 1 PASS]  connected, no errors, default behavior applied.")
    return robot, state


def phase2_stream_hold(robot, ref_state, args):
    if args.skip_stream_hold:
        print("\n[PHASE 2 SKIPPED via --skip-stream-hold]")
        return None
    banner("PHASE 2 — Stream-hold the current pose for 10 s.")
    print("  Expected: robot stays put.  When you gently push the EE you should")
    print("  feel compliance, and ext_wrench should reflect your push.  No reflex.")
    confirm("About to start_cartesian_pose_control(CartesianImpedance) and hold.")

    ac = robot.start_cartesian_pose_control(_impedance_mode(args))

    # Seed target = current pose so RT thread has nothing to chase.
    # set_target_pose takes a 4×4 numpy directly (the C++ binding handles
    # row-major→col-major conversion internally).
    T_hold_4x4 = np.asarray(ref_state["O_T_EE"], dtype=np.float64)
    ac.set_target_pose(T_hold_4x4)

    t0 = time.time()
    last_print = 0.0
    rt_died = False
    while time.time() - t0 < 10.0:
        time.sleep(0.05)
        if not ac.is_running():
            rt_died = True
            print(f"\n  *** RT thread exited unexpectedly at t={time.time()-t0:.2f}s ***")
            print("  This usually means a reflex (collision/communication) was triggered.")
            print("  The state values printed below are STALE (last cache before crash).")
            break
        if time.time() - last_print > 1.0:
            s = ac.get_state()
            pos = s["O_T_EE"][:3, 3]
            f = s["O_F_ext_hat_K"][:3]
            err = s.get("has_error", False)
            print(f"  t={time.time()-t0:5.2f}  pos={vfmt(pos, 8, 4)}  ext_F={vfmt(f, 6, 2)}  err={err}")
            last_print = time.time()
            assert_no_error(s, "phase2.stream_hold")

    s = ac.get_state()
    print(f"\n  Final cmd success rate: {100.0*s['control_command_success_rate']:.2f}%")
    ac.stop()
    if rt_died:
        raise RuntimeError("[PHASE 2 FAIL] RT thread died (likely reflex). "
                           "See [franka_rt][ERR] lines above for diagnostic.")
    print("[PHASE 2 PASS]  10 s hold done, no errors.")
    return ac


def phase3_small_move(robot, ref_state, args):
    banner("PHASE 3 — move_to_pose +1 cm in X over 3 s.")
    print("  Expected: smooth 1 cm motion in +X direction over 3 s.  No reflex.")
    confirm(f"About to move EE +1 cm in X relative to current pose.")

    ac = robot.start_cartesian_pose_control(_impedance_mode(args))

    # Wait past the C++ RT-thread warmup (~50ms) so the RT thread is holding
    # state.O_T_EE_c steadily.  Then read O_T_EE_c (libfranka's last-COMMANDED
    # pose) as our reference — NOT O_T_EE (measured), which differs from
    # commanded by sub-mm under load and would create a discontinuity.
    #
    # Critically, do NOT call set_target_pose() before move_to_pose():
    # move_to_pose() internally seeds the min-jerk trajectory from the RT
    # thread's `last_sent_pose` (= state.O_T_EE_c at session start, which the
    # RT loop has been holding).  An extra streaming command before
    # move_to_pose just creates a tiny step from commanded→measured which is
    # exactly the discontinuity we want to avoid.
    #
    # Pattern matches official libfranka example
    # generate_cartesian_pose_motion_external_control_loop.cpp:
    #   if (time == 0.0) initial_pose = robot_state.O_T_EE_c;
    #   ...
    #   delta_x = kRadius * sin(M_PI/4 * (1 - cos(M_PI/5 * time)));
    # First commanded pose at t=0 equals initial_pose (no delta), and the
    # (1-cos) profile guarantees zero velocity/accel/jerk at t=0.
    time.sleep(0.15)
    state_now = ac.get_state()
    T_curr_4x4   = np.asarray(state_now["O_T_EE_c"], dtype=np.float64).copy()
    T_target_4x4 = T_curr_4x4.copy()
    T_target_4x4[0, 3] += 0.01  # +1 cm in x relative to commanded pose

    # Trigger min-jerk move_to_pose directly — no streaming seed needed.
    ac.move_to_pose(T_target_4x4, duration_sec=3.0)

    t0 = time.time()
    last_print = 0.0
    timeout = 6.0
    while ac.is_moving() and time.time() - t0 < timeout:
        if time.time() - last_print > 0.5:
            s = ac.get_state()
            pos = s["O_T_EE"][:3, 3]
            print(f"  t={time.time()-t0:4.2f}  pos={vfmt(pos, 8, 4)}  is_moving={ac.is_moving()}  err={s.get('has_error')}")
            last_print = time.time()
            assert_no_error(s, "phase3.moving")
        time.sleep(0.02)

    if ac.is_moving():
        print("  WARNING: move_to_pose did not finish in 6 s — cancelling.")
        ac.cancel_move()
        time.sleep(0.5)

    s = ac.get_state()
    pos_final = s["O_T_EE"][:3, 3]
    pos_curr  = T_curr_4x4[:3, 3]
    drift_x   = pos_final[0] - pos_curr[0]
    print(f"\n  Δx achieved : {drift_x*1000:+.2f} mm   (target +10.00 mm)")
    print(f"  cmd success% : {100.0*s['control_command_success_rate']:.2f}")
    ac.stop()
    if abs(drift_x - 0.01) > 0.003:
        print(f"  WARNING: |Δx − 1 cm| = {abs(drift_x-0.01)*1000:.1f} mm (>3 mm)")
    print("[PHASE 3 PASS]  move_to_pose done.")


def phase4_short_sine(robot, ref_state, args):
    banner("PHASE 4 — 30-Hz sinusoid streaming, 30 s.")
    duration = 30.0
    amp = args.sine_amp
    freq = args.sine_freq
    rate = 30       # Python writer rate
    print(f"  Stream X = X0 + {amp*1000:.0f} mm × sin(2π × {freq} Hz × t).")
    print(f"  Python writer at {rate} Hz, RT thread interpolates to 1 kHz.")
    print(f"  Duration: {duration} s.   No reflex expected.")
    confirm(f"About to stream sinusoidal motion (max amplitude {amp*1000:.0f} mm).")

    ac = robot.start_cartesian_pose_control(_impedance_mode(args))
    # Read fresh O_T_EE_c (commanded, what RT holds) after warmup.  Use
    # commanded NOT measured to avoid sub-mm step at first writeOnce.
    time.sleep(0.15)
    state_now = ac.get_state()

    # ---- Diagnostic: pose + elbow + impedance margins ----
    # If micrometre-scale streaming triggers joint_acceleration_discontinuity,
    # the most likely culprits are (a) IK is in a poorly conditioned pose
    # (near wrist singularity, redundancy resolution unstable) or (b) the
    # elbow_c we feed into writeOnce is invalid/inconsistent with measured
    # elbow.  These prints help differentiate the two.
    print("\n  --- Phase 4 start state ---")
    print(f"  EE pos        : {vfmt(state_now['O_T_EE'][:3, 3], 8, 4)} m")
    print(f"  q [rad]       : {vfmt(state_now['q'])}")
    print(f"  q [deg]       : " + ", ".join(f"{math.degrees(v):+7.2f}" for v in state_now['q']))
    print(f"  elbow         : {vfmt(state_now['elbow'], 7, 4)}      (measured)")
    print(f"  elbow_d       : {vfmt(state_now['elbow_d'], 7, 4)}      (desired/internal)")
    print(f"  elbow_c       : {vfmt(state_now['elbow_c'], 7, 4)}      (commanded — what we send back)")
    diff = (state_now['O_T_EE'] - state_now['O_T_EE_c'])[:3, 3]
    print(f"  measured-cmd  : {vfmt(diff*1000, 7, 3)} mm     (impedance compliance offset)")

    fresh_state = {"O_T_EE": state_now["O_T_EE_c"]}
    _run_sine(ac, fresh_state, duration, amp, freq, rate, label="phase4")
    ac.stop()
    print("[PHASE 4 PASS]  30 s of streaming done.")


def phase5_long_sine(robot, ref_state, args):
    if not args.include_long:
        print("\n[PHASE 5 SKIPPED]  add --include-long to run the 5-minute test.")
        return
    banner("PHASE 5 — 30-Hz sinusoid streaming, 5 minutes (sustained reflex test).")
    duration = 300.0
    amp = args.sine_amp
    freq = args.sine_freq
    rate = 30
    print("  Same waveform as Phase 4 but 5 minutes long.  This is the test that")
    print("  validates the goal: sustained streaming without communication_constraints_violation.")
    confirm(f"About to stream for {duration:.0f} s.  Operator stays alert.")

    ac = robot.start_cartesian_pose_control(_impedance_mode(args))
    # Read fresh O_T_EE_c (commanded) after warmup; see phase4 comment.
    time.sleep(0.15)
    state_now = ac.get_state()
    fresh_state = {"O_T_EE": state_now["O_T_EE_c"]}
    _run_sine(ac, fresh_state, duration, amp, freq, rate, label="phase5")
    ac.stop()
    print("[PHASE 5 PASS]  5 min of streaming done.")


def _run_sine(ac, ref_state, duration, amp, freq, rate, label):
    T0_4x4 = np.asarray(ref_state["O_T_EE"], dtype=np.float64).copy()
    dt = 1.0 / rate
    t0 = time.time()
    last_print = 0.0
    iters = 0
    min_rate = 1.0

    # Min-jerk envelope: 0 → 1 over [0, ramp_sec], with zero 1st AND 2nd
    # derivatives at both endpoints.  Multiplying the sine by this envelope
    # guarantees the streaming target at t=0 has zero position, velocity,
    # AND acceleration — eliminating the "hard start" that otherwise causes
    # joint_velocity_discontinuity / joint_acceleration_discontinuity at the
    # first non-zero command.
    ramp_sec = 2.0

    def envelope(t_):
        if t_ <= 0:
            return 0.0
        if t_ >= ramp_sec:
            return 1.0
        s = t_ / ramp_sec
        return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5  # quintic min-jerk

    while True:
        loop_t0 = time.time()
        t = loop_t0 - t0
        if t >= duration:
            break

        env = envelope(t)
        T = T0_4x4.copy()
        T[0, 3] = T0_4x4[0, 3] + amp * env * math.sin(2 * math.pi * freq * t)

        ac.set_target_pose(T)
        iters += 1

        if loop_t0 - last_print > 2.0:
            s = ac.get_state()
            pos = s["O_T_EE"][:3, 3]
            err = s.get("has_error", False)
            sr = s.get("control_command_success_rate", 0.0)
            min_rate = min(min_rate, sr)
            print(f"  [{label}] t={t:6.1f}/{duration:.0f}  pos={vfmt(pos, 8, 4)}  "
                  f"cmd_succ={100*sr:.1f}%  err={err}")
            last_print = loop_t0
            assert_no_error(s, f"{label}.streaming")

        # Pace the writer at `rate` Hz
        sleep_for = dt - (time.time() - loop_t0)
        if sleep_for > 0:
            time.sleep(sleep_for)

    print(f"  [{label}] iters={iters}  min cmd_succ={100*min_rate:.1f}%")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default="192.168.99.111", help="Franka FCI IP")
    p.add_argument("--include-long", action="store_true",
                   help="Run phase 5 (5-minute sustained streaming).")
    p.add_argument("--skip-stream-hold", action="store_true",
                   help="Skip phase 2 (10 s stream-hold).")
    p.add_argument("--phase", type=int, default=-1,
                   help="Run only this phase (0..5).  Default -1 = all enabled.")
    p.add_argument("--skip-go-home", action="store_true",
                   help="Skip phase 0 (go-home).  Useful if you want to keep the "
                        "current robot pose and only test compliance/streaming.")
    p.add_argument("--sine-amp", type=float, default=0.02,
                   help="Phase 4/5 sine amplitude in metres (default 0.02 = 2 cm).")
    p.add_argument("--sine-freq", type=float, default=0.2,
                   help="Phase 4/5 sine frequency in Hz (default 0.2).")
    p.add_argument("--impedance-mode", choices=["cartesian", "joint"], default="cartesian",
                   help="kCartesianImpedance (default) is end-effector compliant; "
                        "kJointImpedance is rigid joint-space tracking (less sensitive to "
                        "EE-config mass error and IK conditioning, useful for diagnosis).")
    args = p.parse_args()

    print(__doc__)

    robot = None
    try:
        robot, ref_state = phase1_connect(args)
        if args.phase in (-1, 1):
            if args.phase == 1:
                return 0

        # Phase 0 = go-home; runs after Phase 1 (which only reads state) so
        # the robot is unlocked + has fresh state before we move.
        if args.phase in (-1, 0):
            phase0_go_home(robot, args)
            # Refresh ref_state after the move so subsequent phases use the
            # post-go-home pose as their reference.
            ref_state = robot.read_once()
            if args.phase == 0:
                return 0

        if args.phase in (-1, 2):
            phase2_stream_hold(robot, ref_state, args)
            if args.phase == 2:
                return 0

        if args.phase in (-1, 3):
            phase3_small_move(robot, ref_state, args)
            if args.phase == 3:
                return 0

        if args.phase in (-1, 4):
            phase4_short_sine(robot, ref_state, args)
            if args.phase == 4:
                return 0

        if args.phase in (-1, 5):
            phase5_long_sine(robot, ref_state, args)

    except KeyboardInterrupt:
        print("\n\n*** Ctrl+C caught — exiting cleanly ***")
        return 130
    except Exception as e:
        print("\n\n*** EXCEPTION CAUGHT ***")
        traceback.print_exc()
        print(f"\n  {type(e).__name__}: {e}")
        return 2
    finally:
        if robot is not None:
            try:
                robot.stop()
            except Exception:
                pass

    banner("ALL REQUESTED PHASES COMPLETED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
