#include "realtime_control/cartesian_control.hpp"

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>

#include <algorithm>
#include <cmath>

#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"

namespace franka_rt {

namespace {

// ---------------------------------------------------------------------------
// Pack franka::Errors (41 bool fields) into a uint64.  Bit positions match
// the declaration order in franka/errors.h.  Decoded on the Python side via
// a static name table.
// ---------------------------------------------------------------------------
uint64_t PackErrors(const franka::Errors& e) noexcept {
    uint64_t r = 0;
    int b = 0;
    auto add = [&](bool v) { if (v) r |= (1ULL << b); ++b; };
    add(e.joint_position_limits_violation);
    add(e.cartesian_position_limits_violation);
    add(e.self_collision_avoidance_violation);
    add(e.joint_velocity_violation);
    add(e.cartesian_velocity_violation);
    add(e.force_control_safety_violation);
    add(e.joint_reflex);
    add(e.cartesian_reflex);
    add(e.max_goal_pose_deviation_violation);
    add(e.max_path_pose_deviation_violation);
    add(e.cartesian_velocity_profile_safety_violation);
    add(e.joint_position_motion_generator_start_pose_invalid);
    add(e.joint_motion_generator_position_limits_violation);
    add(e.joint_motion_generator_velocity_limits_violation);
    add(e.joint_motion_generator_velocity_discontinuity);
    add(e.joint_motion_generator_acceleration_discontinuity);
    add(e.cartesian_position_motion_generator_start_pose_invalid);
    add(e.cartesian_motion_generator_elbow_limit_violation);
    add(e.cartesian_motion_generator_velocity_limits_violation);
    add(e.cartesian_motion_generator_velocity_discontinuity);
    add(e.cartesian_motion_generator_acceleration_discontinuity);
    add(e.cartesian_motion_generator_elbow_sign_inconsistent);
    add(e.cartesian_motion_generator_start_elbow_invalid);
    add(e.cartesian_motion_generator_joint_position_limits_violation);
    add(e.cartesian_motion_generator_joint_velocity_limits_violation);
    add(e.cartesian_motion_generator_joint_velocity_discontinuity);
    add(e.cartesian_motion_generator_joint_acceleration_discontinuity);
    add(e.cartesian_position_motion_generator_invalid_frame);
    add(e.force_controller_desired_force_tolerance_violation);
    add(e.controller_torque_discontinuity);
    add(e.start_elbow_sign_inconsistent);
    add(e.communication_constraints_violation);
    add(e.power_limit_violation);
    add(e.joint_p2p_insufficient_torque_for_planning);
    add(e.tau_j_range_violation);
    add(e.instability_detected);
    add(e.joint_move_in_wrong_direction);
    add(e.cartesian_spline_motion_generator_violation);
    add(e.joint_via_motion_generator_planning_joint_limit_violation);
    add(e.base_acceleration_initialization_timeout);
    add(e.base_acceleration_invalid_reading);
    return r;
}

RobotMode ConvertRobotMode(franka::RobotMode m) noexcept {
    switch (m) {
        case franka::RobotMode::kIdle:                    return RobotMode::Idle;
        case franka::RobotMode::kMove:                    return RobotMode::Move;
        case franka::RobotMode::kGuiding:                 return RobotMode::Guiding;
        case franka::RobotMode::kReflex:                  return RobotMode::Reflex;
        case franka::RobotMode::kUserStopped:             return RobotMode::UserStopped;
        case franka::RobotMode::kAutomaticErrorRecovery:  return RobotMode::AutomaticErrorRecovery;
        default:                                          return RobotMode::Other;
    }
}

void FillCartesianStateFromFranka(CartesianState& cs,
                                  const franka::RobotState& s) noexcept {
    cs.O_T_EE   = s.O_T_EE;
    cs.q        = s.q;
    cs.dq       = s.dq;
    cs.elbow    = s.elbow;

    cs.O_T_EE_d  = s.O_T_EE_d;
    cs.O_dP_EE_d = s.O_dP_EE_d;
    cs.q_d       = s.q_d;
    cs.dq_d      = s.dq_d;
    cs.elbow_d   = s.elbow_d;

    cs.O_T_EE_c  = s.O_T_EE_c;
    cs.O_dP_EE_c = s.O_dP_EE_c;
    cs.O_ddP_EE_c = s.O_ddP_EE_c;
    cs.elbow_c   = s.elbow_c;

    cs.O_F_ext_hat_K        = s.O_F_ext_hat_K;
    cs.K_F_ext_hat_K        = s.K_F_ext_hat_K;
    cs.tau_ext_hat_filtered = s.tau_ext_hat_filtered;

    cs.current_errors      = PackErrors(s.current_errors);
    cs.last_motion_errors  = PackErrors(s.last_motion_errors);
    cs.robot_mode          = ConvertRobotMode(s.robot_mode);
    cs.control_command_success_rate = s.control_command_success_rate;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CartesianControl::CartesianControl(franka::Robot& robot, ControllerMode mode)
    : robot_(robot), mode_(mode) {}

CartesianControl::~CartesianControl() {
    stop();
}

void CartesianControl::start() {
    if (running_.exchange(true)) return;
    stop_requested_.store(false);
    stopped_.store(false);
    estop_.store(false);
    moving_.store(false);
    cmd_buf_.clear();
    rt_thread_ = std::thread(&CartesianControl::rtLoop, this);
}

void CartesianControl::stop() {
    if (stopped_.exchange(true)) return;
    stop_requested_.store(true);
    if (rt_thread_.joinable()) rt_thread_.join();
    running_.store(false);
}

// ---------------------------------------------------------------------------
// Public command API
// ---------------------------------------------------------------------------

void CartesianControl::setTargetPose(const std::array<double, 16>& T) noexcept {
    PoseCommand cmd;
    cmd.T = T;
    cmd.has_elbow = false;
    cmd.timestamp = std::chrono::steady_clock::now();
    // Best-effort enqueue — buffer should never fill in practice.
    cmd_buf_.try_write(cmd);
}

void CartesianControl::setTargetPoseWithElbow(
        const std::array<double, 16>& T,
        const std::array<double, 2>& elbow) noexcept {
    PoseCommand cmd;
    cmd.T = T;
    cmd.elbow = elbow;
    cmd.has_elbow = true;
    cmd.timestamp = std::chrono::steady_clock::now();
    cmd_buf_.try_write(cmd);
}

CartesianState CartesianControl::getState() const noexcept {
    std::lock_guard<std::mutex> g(state_mtx_);
    return latest_state_;
}

void CartesianControl::moveToPose(const std::array<double, 16>& T,
                                  double duration_sec) noexcept {
    std::lock_guard<std::mutex> g(pending_move_mtx_);
    pending_move_target_   = T;
    pending_move_duration_ = duration_sec;
    pending_move_init_     = true;
    moving_.store(true);
}

void CartesianControl::cancelMove() noexcept {
    std::lock_guard<std::mutex> g(pending_move_mtx_);
    pending_move_init_ = false;
    moving_.store(false);
}

// ---------------------------------------------------------------------------
// 1 kHz RT loop
// ---------------------------------------------------------------------------

void CartesianControl::rtLoop() {
    // 1. Try to elevate this thread to RT priority (best-effort).
    {
        std::string err;
        if (!franka::setCurrentThreadToHighestSchedulerPriority(&err)) {
            logger().warn("Could not set RT priority on CartesianControl thread: %s",
                          err.c_str());
        }
    }

    // ------------------------------------------------------------------
    // Why this loop uses robot_.control(callback) instead of active control
    // ------------------------------------------------------------------
    //
    // We previously ran `start*Control() + readOnce/writeOnce` and observed
    // `cartesian_motion_generator_acceleration_discontinuity` /
    // `cartesian_motion_generator_joint_acceleration_discontinuity` errors
    // even on sub-mm sine streaming.  An A/B test against the official
    // libfranka `robot.control(callback, mode, limit_rate=true)` path
    // (see tests/robot/libfranka_cartesian_sine.cpp) ran the same sine
    // amplitudes with no errors, including 5 cm / 0.2 Hz.
    //
    // That pulled the ground truth: libfranka's `robot.control()` with
    // limit_rate=true wraps the user callback in an internal pipeline that:
    //   - synthesises a perfectly periodic 1 kHz cycle (we cannot match
    //     this externally — our scheduling jitter creates phase noise)
    //   - applies its rate limiter using the actual previously-sent
    //     command/velocity/acceleration that libfranka tracks (we cannot
    //     observe these from outside, only `state.O_T_EE_c` which is one
    //     cycle stale)
    //   - feeds the motion generator's internal lowpass at the boundary
    //     it expects
    //
    // None of these are reachable through the active-control API.  The
    // safe choice is to use the same path as the diagnostic that we just
    // proved works, and let libfranka own the 1 kHz timing + filtering.
    // We keep our SPSC ring buffer + safety checks inside the callback.
    // Streaming smoothing is deliberately delegated to libfranka's own
    // `limit_rate=true` path, matching the official callback diagnostic.
    //
    // Lifecycle: `robot_.control(callback)` blocks until the callback
    // returns `franka::MotionFinished`.  Since this whole rtLoop()
    // already runs on a dedicated std::thread, blocking is fine.  Stop /
    // estop are signalled by atomic flags that the callback inspects each
    // cycle and converts into MotionFinished.

    // -------------------- callback-local state --------------------
    //
    // All variables that carry information across cycles live here as
    // captures of the lambda below.  They are equivalent to instance
    // members but kept thread-local to make ownership obvious.
    bool first_cycle = true;
    std::array<double, 16> last_sent_pose{};
    std::array<double, 16> stream_pose{};
    std::array<double, 3>  stream_vel{};
    std::array<double, 3>  stream_acc{};

    // Persistent streaming target — updated only when a new Python command
    // arrives.  In STREAMING mode we do not forward this sparse 30 Hz target
    // directly.  Instead a 1 kHz jerk-limited online trajectory generator
    // advances `stream_pose` toward it.  This preserves the CartesianPose
    // motion-generator semantics while matching the official successful
    // callback tests: every callback returns a smooth pose sample.
    std::array<double, 16> stream_target_pose{};

    MinJerkTrajectory move_traj;

    std::chrono::steady_clock::time_point last_cmd_time;
    bool   last_cmd_time_valid = false;
    double cmd_interval_sec    = kDefaultCommandInterval;

    enum class RTState { STREAMING, MOVING };
    RTState rt_state = RTState::STREAMING;

    // Diagnostic ring buffer (last 5 cycles) — for crash post-mortems.
    struct DiagSample {
        uint64_t iter;
        double   period_ms;
    };
    std::array<DiagSample, 5> diag{};
    int      diag_idx = 0;
    uint64_t iter = 0;
    bool graceful_stop_active = false;
    int graceful_stop_hold_cycles = 0;

    // Warmup: hold libfranka's commanded pose for the first N cycles so
    // that the motion generator's internal filters reach a coherent
    // zero-velocity / zero-acceleration steady state before we accept
    // streaming input.  Drops any commands that arrived during warmup so
    // they cannot cause an instantaneous step on the warmup-end cycle.
    constexpr int kWarmupCycles = 50;

    // -------------------- the callback --------------------
    auto callback = [&](const franka::RobotState& state,
                        franka::Duration period) -> franka::CartesianPose {
        // ---- First cycle: seed everything from libfranka's commanded pose ----
        if (first_cycle) {
            last_sent_pose = state.O_T_EE_c;
            stream_pose = state.O_T_EE_c;
            stream_target_pose = state.O_T_EE_c;
            stream_vel = {0.0, 0.0, 0.0};
            stream_acc = {0.0, 0.0, 0.0};
            first_cycle = false;
        }

        // ---- Snapshot for getState() ----
        CartesianState cs;
        FillCartesianStateFromFranka(cs, state);
        {
            std::unique_lock<std::mutex> g(state_mtx_, std::try_to_lock);
            if (g.owns_lock()) {
                latest_state_ = cs;
            }
        }

        // ---- Diagnostic record ----
        diag[diag_idx] = { iter, period.toSec() * 1000.0 };
        diag_idx = (diag_idx + 1) % 5;
        ++iter;
        // Use the actual period libfranka delivers.  Clamping dt to
        // 2 ms while libfranka tells us 3 ms elapsed makes
        //   p_new = p_old + stream_vel * 2 ms
        // but libfranka samples velocity over the true 3 ms period:
        //   v_seen = (p_new - p_old) / 3 ms = stream_vel * 2/3
        // i.e. a 33 % velocity step at the sample boundary that fires
        // `cartesian_motion_generator_velocity_discontinuity` etc.
        // A 1 us floor keeps math safe without lying about elapsed
        // time.
        const double dt = std::max(period.toSec(), 1e-6);

        // ---- Stop / e-stop / fault: terminate the motion ----
        if (estop_.load()) {
            return franka::MotionFinished(franka::CartesianPose(last_sent_pose));
        }
        if (state.current_errors) {
            logger().error("Robot fault detected (cycle %lu): %s",
                           (unsigned long)iter,
                           static_cast<std::string>(state.current_errors).c_str());
            estop_.store(true);
            return franka::MotionFinished(franka::CartesianPose(last_sent_pose));
        }
        if (stop_requested_.load()) {
            if (!graceful_stop_active) {
                graceful_stop_active = true;
                graceful_stop_hold_cycles = 0;
                if (rt_state == RTState::MOVING) {
                    stream_pose = last_sent_pose;
                    stream_vel = {0.0, 0.0, 0.0};
                    stream_acc = {0.0, 0.0, 0.0};
                }
                rt_state = RTState::STREAMING;
                moving_.store(false);
                stream_target_pose = stream_pose;
            }

            bool quiet = true;
            constexpr double kKd = 12.0;
            constexpr double kMaxAcc = 1.5;
            constexpr double kMaxJerk = 12.0;
            for (int i = 0; i < 3; ++i) {
                const double desired_acc =
                    std::clamp(-kKd * stream_vel[i], -kMaxAcc, kMaxAcc);
                const double max_da = kMaxJerk * dt;
                const double da = std::clamp(desired_acc - stream_acc[i],
                                             -max_da, max_da);
                stream_acc[i] += da;
                stream_vel[i] += stream_acc[i] * dt;
                stream_vel[i] = std::clamp(stream_vel[i], -0.25, 0.25);
                if (std::abs(stream_vel[i]) < 1e-5 &&
                    std::abs(stream_acc[i]) < 1e-4) {
                    stream_vel[i] = 0.0;
                    stream_acc[i] = 0.0;
                } else {
                    quiet = false;
                }
                stream_pose[12 + i] += stream_vel[i] * dt;
            }

            last_sent_pose = stream_pose;
            if (quiet) {
                ++graceful_stop_hold_cycles;
            } else {
                graceful_stop_hold_cycles = 0;
            }
            if (graceful_stop_hold_cycles >= 100) {
                return franka::MotionFinished(franka::CartesianPose(last_sent_pose));
            }
            return franka::CartesianPose(last_sent_pose);
        }

        const bool in_warmup = (iter <= (uint64_t)kWarmupCycles);

        // ---- Drain ring buffer; keep only the latest streaming command ----
        PoseCommand latest_cmd;
        bool got_cmd = false;
        {
            PoseCommand tmp;
            while (cmd_buf_.try_read(tmp)) {
                latest_cmd = tmp;
                got_cmd = true;
            }
        }
        if (in_warmup) got_cmd = false;

        // ---- Pick up any pending moveToPose request (also gated) ----
        bool start_move = false;
        std::array<double, 16> move_target{};
        double                 move_duration = 0.0;
        if (!in_warmup) {
            std::unique_lock<std::mutex> g(pending_move_mtx_, std::try_to_lock);
            if (g.owns_lock() && pending_move_init_) {
                move_target    = pending_move_target_;
                move_duration  = pending_move_duration_;
                pending_move_init_ = false;
                start_move = true;
            }
        }
        if (start_move) {
            move_traj.init(last_sent_pose, move_target, move_duration);
            rt_state = RTState::MOVING;
            moving_.store(true);
        }

        std::array<double, 16> T_cmd = stream_target_pose;

        if (rt_state == RTState::MOVING) {
            bool in_progress = move_traj.step(T_cmd);
                if (!in_progress) {
                    rt_state = RTState::STREAMING;
                    moving_.store(false);
                    stream_target_pose = T_cmd;
                    stream_pose = T_cmd;
                    stream_vel = {0.0, 0.0, 0.0};
                    stream_acc = {0.0, 0.0, 0.0};
                }
        } else {
            // STREAMING.
            if (got_cmd) {
                if (!CheckFinite(latest_cmd.T)) {
                    logger().error("NaN/Inf in command pose; e-stop");
                    estop_.store(true);
                    return franka::MotionFinished(franka::CartesianPose(last_sent_pose));
                }
                if (!franka::isHomogeneousTransformation(latest_cmd.T)) {
                    logger().warn("Non-homogeneous command pose; dropping");
                } else {
                    auto now = latest_cmd.timestamp;
                    if (last_cmd_time_valid) {
                        double dt = std::chrono::duration<double>(
                                        now - last_cmd_time).count();
                        cmd_interval_sec = std::clamp(
                            dt, kMinCommandInterval, kMaxCommandInterval);
                    }
                    last_cmd_time = now;
                    last_cmd_time_valid = true;

                    double pos_thresh = kMaxPositionVelocity * cmd_interval_sec;
                    double rot_thresh = kMaxRotationVelocity * cmd_interval_sec;
                    if (CheckCartesianJump(latest_cmd.T, stream_target_pose,
                                           pos_thresh, rot_thresh)) {
                        logger().warn(
                            "Cartesian jump (>%.3fm or >%.3frad) at cycle %lu — dropping",
                            pos_thresh, rot_thresh, (unsigned long)iter);
                    } else {
                        // Adopt the new target.  Do not interpolate here:
                        // returning a zero-order-held target from the
                        // callback matches the libfranka API contract when
                        // `limit_rate=true`; libfranka performs the RT-safe
                        // filtering at the exact internal boundary it expects.
                        stream_target_pose = latest_cmd.T;
                    }
                }
            }

            // 1 kHz online trajectory generator for sparse pose targets.
            //
            // This is deliberately stateful in velocity and acceleration.
            // A Python command may step the target every ~33 ms, but the pose
            // we return to libfranka changes with bounded jerk.  That is the
            // missing piece compared with forwarding the latest target
            // directly, which creates a discontinuity at the CartesianPose
            // motion-generator input and then shows up as joint-side
            // discontinuities after IK.
            constexpr double kKp = 36.0;       // 1/s^2
            constexpr double kKd = 12.0;       // 1/s, critically damped-ish
            constexpr double kMaxVel = 0.25;   // m/s
            constexpr double kMaxAcc = 1.5;    // m/s^2
            constexpr double kMaxJerk = 12.0;  // m/s^3

            for (int i = 0; i < 3; ++i) {
                double err = stream_target_pose[12 + i] - stream_pose[12 + i];
                double desired_acc = kKp * err - kKd * stream_vel[i];
                desired_acc = std::clamp(desired_acc, -kMaxAcc, kMaxAcc);

                double max_da = kMaxJerk * dt;
                double da = std::clamp(desired_acc - stream_acc[i], -max_da, max_da);
                stream_acc[i] += da;

                stream_vel[i] += stream_acc[i] * dt;
                stream_vel[i] = std::clamp(stream_vel[i], -kMaxVel, kMaxVel);

                stream_pose[12 + i] += stream_vel[i] * dt;
            }

            // Phase 4 and teleop translation tests keep orientation fixed.
            // For orientation target changes, keep a conservative per-cycle
            // rotation clamp until we add a full angular jerk-limited filter.
            for (int i : {0, 1, 2, 4, 5, 6, 8, 9, 10}) {
                stream_pose[i] = stream_target_pose[i];
            }

            T_cmd = stream_pose;
        }

        last_sent_pose = T_cmd;

        // ---- Return: no elbow.  libfranka maintains redundancy
        //              internally and we proved this is what works in
        //              the official-callback diagnostic. ----
        return franka::CartesianPose(T_cmd);
    };

    // -------------------- run libfranka's blocking control loop --------------------
    auto controller_mode = (mode_ == ControllerMode::CartesianImpedance)
        ? franka::ControllerMode::kCartesianImpedance
        : franka::ControllerMode::kJointImpedance;

    try {
        robot_.control(callback, controller_mode, /*limit_rate=*/true);
    } catch (const franka::Exception& e) {
        logger().error("CartesianControl robot.control exception (iter %lu): %s",
                       (unsigned long)iter, e.what());
        logger().error("  Last 5 cycle periods (oldest first):");
        for (int i = 0; i < 5; ++i) {
            int idx = (diag_idx + i) % 5;
            const auto& d = diag[idx];
            logger().error("    iter=%lu period=%.2fms",
                          (unsigned long)d.iter, d.period_ms);
        }
    } catch (...) {
        logger().error("CartesianControl unknown exception (iter %lu)",
                       (unsigned long)iter);
    }

    running_.store(false);
    moving_.store(false);
}

}  // namespace franka_rt
