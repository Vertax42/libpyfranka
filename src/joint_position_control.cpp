#include "realtime_control/joint_position_control.hpp"

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>

#include <algorithm>
#include <cmath>

#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"

namespace franka_rt {

namespace {

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

void FillJointStateFromFranka(JointState& js,
                              const franka::RobotState& s) noexcept {
    js.q                    = s.q;
    js.dq                   = s.dq;
    js.tau_J                = s.tau_J;
    js.dtau_J               = s.dtau_J;
    js.tau_ext_hat_filtered = s.tau_ext_hat_filtered;
    js.O_T_EE               = s.O_T_EE;

    js.q_d     = s.q_d;
    js.dq_d    = s.dq_d;
    js.ddq_d   = s.ddq_d;
    js.tau_J_d = s.tau_J_d;

    js.O_F_ext_hat_K = s.O_F_ext_hat_K;
    js.K_F_ext_hat_K = s.K_F_ext_hat_K;

    js.current_errors      = PackErrors(s.current_errors);
    js.last_motion_errors  = PackErrors(s.last_motion_errors);
    js.robot_mode          = ConvertRobotMode(s.robot_mode);
    js.control_command_success_rate = s.control_command_success_rate;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

JointPositionControl::JointPositionControl(franka::Robot& robot)
    : robot_(robot) {}

JointPositionControl::~JointPositionControl() {
    stop();
}

void JointPositionControl::start() {
    if (running_.exchange(true)) return;
    stop_requested_.store(false);
    stopped_.store(false);
    estop_.store(false);
    moving_.store(false);
    cmd_buf_.clear();
    rt_thread_ = std::thread(&JointPositionControl::rtLoop, this);
}

void JointPositionControl::stop() {
    if (stopped_.exchange(true)) return;
    stop_requested_.store(true);
    if (rt_thread_.joinable()) rt_thread_.join();
    running_.store(false);
}

// ---------------------------------------------------------------------------
// Public command API
// ---------------------------------------------------------------------------

void JointPositionControl::setTargetJoints(const std::array<double, 7>& q) noexcept {
    JointCommand cmd;
    cmd.q = q;
    cmd.timestamp = std::chrono::steady_clock::now();
    cmd_buf_.try_write(cmd);
}

JointState JointPositionControl::getState() const noexcept {
    std::lock_guard<std::mutex> g(state_mtx_);
    return latest_state_;
}

void JointPositionControl::moveToJoints(const std::array<double, 7>& q,
                                        double duration_sec) noexcept {
    std::lock_guard<std::mutex> g(pending_move_mtx_);
    pending_move_target_   = q;
    pending_move_duration_ = duration_sec;
    pending_move_init_     = true;
    moving_.store(true);
}

void JointPositionControl::cancelMove() noexcept {
    std::lock_guard<std::mutex> g(pending_move_mtx_);
    pending_move_init_ = false;
    moving_.store(false);
}

// ---------------------------------------------------------------------------
// 1 kHz RT loop
// ---------------------------------------------------------------------------

void JointPositionControl::rtLoop() {
    {
        std::string err;
        if (!franka::setCurrentThreadToHighestSchedulerPriority(&err)) {
            logger().warn("Could not set RT priority on JointPositionControl thread: %s",
                          err.c_str());
        }
    }

    // -------------------- callback-local state --------------------
    bool first_cycle = true;
    std::array<double, 7> last_sent_q{};

    // Continuous online trajectory generator (OTG) state.  These three
    // arrays are NEVER reset during streaming — they form a 2nd-order
    // jerk-limited filter that follows `stream_target_q`.  Every Python
    // command updates the SETPOINT only; the filter integrates smoothly
    // toward it at 1 kHz.
    //
    // This is the design we converged on for CartesianControl after
    // segment-based interpolation kept producing acceleration impulses
    // at re-init boundaries.  Joint commands feed libfranka's joint
    // motion generator directly (no IK), so the same continuity matters
    // here — without it, `joint_motion_generator_velocity_discontinuity`
    // / `joint_motion_generator_acceleration_discontinuity` fire on
    // sparse-input streams.
    std::array<double, 7> stream_q{};
    std::array<double, 7> stream_dq{};
    std::array<double, 7> stream_ddq{};
    std::array<double, 7> stream_target_q{};

    // OTG gains — critically damped 6 rad/s natural frequency.  Shared
    // with CartesianControl translation OTG so streaming behaviour feels
    // consistent across modes.
    constexpr double kKp      = 36.0;   // 1/s²
    constexpr double kKd      = 12.0;   // 1/s
    constexpr double kMaxVel  = 1.5;    // rad/s — well below libfranka per-joint limits
    constexpr double kMaxAcc  = 6.0;    // rad/s²
    constexpr double kMaxJerk = 50.0;   // rad/s³

    // moveToJoints state — explicit min-jerk segment from current OTG
    // pose to target with a defined duration.  When the segment finishes,
    // we sync OTG state (q ← endpoint, dq=ddq=0) and resume streaming.
    struct MoveState {
        std::array<double, 7> q_start{};
        std::array<double, 7> q_end{};
        double duration_sec = 0.0;
        double t = 0.0;
        bool active = false;
    } move_state{};

    std::chrono::steady_clock::time_point last_cmd_time;
    bool   last_cmd_time_valid = false;
    double cmd_interval_sec    = kDefaultCommandInterval;

    enum class RTState { STREAMING, MOVING };
    RTState rt_state = RTState::STREAMING;

    struct DiagSample {
        uint64_t iter;
        double   period_ms;
    };
    std::array<DiagSample, 5> diag{};
    int      diag_idx = 0;
    uint64_t iter = 0;
    bool graceful_stop_active = false;
    int graceful_stop_hold_cycles = 0;

    constexpr int kWarmupCycles = 50;

    // Min-jerk basis s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ for moveToJoints.
    auto min_jerk_s = [](double tau) {
        if (tau <= 0.0) return 0.0;
        if (tau >= 1.0) return 1.0;
        double t2 = tau * tau, t3 = t2 * tau;
        return 10.0 * t3 - 15.0 * t3 * tau + 6.0 * t3 * t2;
    };

    auto callback = [&](const franka::RobotState& state,
                        franka::Duration period) -> franka::JointPositions {
        if (first_cycle) {
            last_sent_q     = state.q_d;
            stream_q        = state.q_d;
            stream_target_q = state.q_d;
            stream_dq       = {0, 0, 0, 0, 0, 0, 0};
            stream_ddq      = {0, 0, 0, 0, 0, 0, 0};
            first_cycle = false;
        }

        // Snapshot to Python.  try_lock() — if Python is mid-read we
        // simply skip this update.  At 1 kHz a single missed update is
        // invisible to consumers; in exchange the RT loop can never be
        // stalled by a slow Python thread (no priority inversion).
        JointState js;
        FillJointStateFromFranka(js, state);
        {
            std::unique_lock<std::mutex> g(state_mtx_, std::try_to_lock);
            if (g.owns_lock()) {
                latest_state_ = js;
            }
        }

        diag[diag_idx] = { iter, period.toSec() * 1000.0 };
        diag_idx = (diag_idx + 1) % 5;
        ++iter;
        // Use the actual period libfranka tells us — DO NOT clamp.
        // If we get a 3 ms scheduler-jitter cycle (libfranka still
        // honours the request, just delivers it late), and we
        // integrated the OTG as if only 2 ms elapsed, then
        //   q_new = q_old + stream_dq * 2 ms
        // but libfranka samples velocity over the real 3 ms period:
        //   v_seen = (q_new - q_old) / 3 ms = stream_dq * 2/3
        // i.e. a 33 % velocity step at the sample boundary, which
        // fires `joint_motion_generator_velocity_discontinuity` +
        // `_acceleration_discontinuity` after enough jitter samples.
        // A tiny floor at 1 us keeps division/multiplication safe
        // without ever lying about elapsed time.
        const double dt = std::max(period.toSec(), 1e-6);

        if (estop_.load()) {
            return franka::MotionFinished(franka::JointPositions(last_sent_q));
        }
        if (state.current_errors) {
            logger().error("Robot fault detected (cycle %lu): %s",
                           (unsigned long)iter,
                           static_cast<std::string>(state.current_errors).c_str());
            estop_.store(true);
            return franka::JointPositions(last_sent_q);
        }
        if (stop_requested_.load()) {
            if (!graceful_stop_active) {
                graceful_stop_active = true;
                graceful_stop_hold_cycles = 0;
                if (rt_state == RTState::MOVING) {
                    stream_q = last_sent_q;
                    stream_dq = {0, 0, 0, 0, 0, 0, 0};
                    stream_ddq = {0, 0, 0, 0, 0, 0, 0};
                }
                move_state.active = false;
                rt_state = RTState::STREAMING;
                moving_.store(false);
                stream_target_q = stream_q;
            }

            bool quiet = true;
            for (int i = 0; i < 7; ++i) {
                const double desired_acc =
                    std::clamp(-kKd * stream_dq[i], -kMaxAcc, kMaxAcc);
                const double max_da = kMaxJerk * dt;
                const double da = std::clamp(desired_acc - stream_ddq[i],
                                             -max_da, max_da);
                stream_ddq[i] += da;
                stream_dq[i] += stream_ddq[i] * dt;
                stream_dq[i] = std::clamp(stream_dq[i], -kMaxVel, kMaxVel);
                if (std::abs(stream_dq[i]) < 1e-4 &&
                    std::abs(stream_ddq[i]) < 1e-3) {
                    stream_dq[i] = 0.0;
                    stream_ddq[i] = 0.0;
                } else {
                    quiet = false;
                }
                stream_q[i] += stream_dq[i] * dt;
            }

            std::array<double, 7> cmd_q = stream_q;
            ClampJointPosition(cmd_q, last_sent_q);
            stream_q = cmd_q;
            last_sent_q = cmd_q;

            if (quiet) {
                ++graceful_stop_hold_cycles;
            } else {
                graceful_stop_hold_cycles = 0;
            }
            if (graceful_stop_hold_cycles >= 100) {
                return franka::MotionFinished(franka::JointPositions(last_sent_q));
            }
            return franka::JointPositions(last_sent_q);
        }

        const bool in_warmup = (iter <= (uint64_t)kWarmupCycles);

        // Drain ring buffer; keep only the latest streaming command.
        JointCommand latest_cmd;
        bool got_cmd = false;
        {
            JointCommand tmp;
            while (cmd_buf_.try_read(tmp)) {
                latest_cmd = tmp;
                got_cmd = true;
            }
        }
        if (in_warmup) got_cmd = false;

        // Pick up any pending moveToJoints request.  try_lock() again —
        // if Python happens to be inside moveToJoints() this cycle we
        // pick the move up next cycle instead.  A 1 ms delay on a move
        // request is far better than blocking the 1 kHz loop.
        bool start_move = false;
        std::array<double, 7> move_target{};
        double                move_duration = 0.0;
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
            // Clamp the user-provided target to FR3 hard joint limits.
            // The min-jerk segment is parameterised once at start, so a
            // target outside [kFR3JointMin, kFR3JointMax] would otherwise
            // drive the trajectory toward a clamp wall and ClampJointPosition
            // below would have to fight it every cycle.
            for (int i = 0; i < 7; ++i) {
                move_target[i] = std::clamp(move_target[i],
                                            kFR3JointMin[i],
                                            kFR3JointMax[i]);
            }

            // Auto-compute duration when the user passed 0 (or anything
            // smaller than the safety floor).  Min-jerk peak velocity is
            // 1.875 × |Δq| / T, so T = 1.875 × max|Δq| / vel_budget.
            // We use a conservative vel_budget of 0.5 rad/s (≈30 °/s)
            // and a 1.0 s floor — this matches typical "go-home" feel
            // without ever surprising the operator with a fast motion.
            // If the user supplies a positive duration we honour it,
            // but enforce the same kMinMoveDuration floor so is_moving()
            // stays meaningful.
            constexpr double kAutoVelBudget   = 0.5;   // rad/s
            constexpr double kMinMoveDuration = 1.0;   // s
            double max_dq = 0.0;
            for (int i = 0; i < 7; ++i) {
                max_dq = std::max(max_dq, std::abs(move_target[i] - stream_q[i]));
            }
            double auto_dur = 1.875 * max_dq / kAutoVelBudget;
            double dur = (move_duration > 0.0) ? move_duration : auto_dur;
            dur = std::max(dur, kMinMoveDuration);

            move_state.q_start      = stream_q;
            move_state.q_end        = move_target;
            move_state.duration_sec = dur;
            move_state.t            = 0.0;
            move_state.active       = true;
            rt_state = RTState::MOVING;
            moving_.store(true);
        }

        std::array<double, 7> cmd_q = stream_q;

        if (rt_state == RTState::MOVING) {
            move_state.t += dt;
            double tau = move_state.t / move_state.duration_sec;
            double s = min_jerk_s(tau);
            for (int i = 0; i < 7; ++i) {
                cmd_q[i] = move_state.q_start[i]
                         + s * (move_state.q_end[i] - move_state.q_start[i]);
            }
            if (move_state.t >= move_state.duration_sec) {
                move_state.active = false;
                rt_state = RTState::STREAMING;
                moving_.store(false);
                // Sync OTG state to the end of the move so streaming
                // continues smoothly from here.
                stream_q        = move_state.q_end;
                stream_target_q = move_state.q_end;
                stream_dq       = {0, 0, 0, 0, 0, 0, 0};
                stream_ddq      = {0, 0, 0, 0, 0, 0, 0};
                cmd_q = stream_q;
            }
        } else {
            // STREAMING.
            if (got_cmd) {
                if (!CheckFinite(latest_cmd.q)) {
                    logger().error("NaN/Inf in joint command; e-stop");
                    estop_.store(true);
                    return franka::MotionFinished(franka::JointPositions(last_sent_q));
                }
                auto now = latest_cmd.timestamp;
                if (last_cmd_time_valid) {
                    double interval = std::chrono::duration<double>(
                                          now - last_cmd_time).count();
                    cmd_interval_sec = std::clamp(
                        interval, kMinCommandInterval, kMaxCommandInterval);
                }
                last_cmd_time = now;
                last_cmd_time_valid = true;

                // Adaptive jump check — drop commands that step further
                // than kMaxJointVelocity (rad/s) over the observed
                // interval.  Mirrors the Cartesian jump check.
                double thresh = kMaxJointVelocityCmd * cmd_interval_sec;
                bool jump = false;
                for (int i = 0; i < 7; ++i) {
                    if (std::abs(latest_cmd.q[i] - stream_target_q[i]) > thresh) {
                        jump = true;
                        break;
                    }
                }
                if (jump) {
                    logger().warn(
                        "Joint jump (>%.3frad on some axis) at cycle %lu — dropping",
                        thresh, (unsigned long)iter);
                } else {
                    stream_target_q = latest_cmd.q;
                }
            }

            // 1 kHz online trajectory generator: PD toward target with
            // bounded jerk + acc + vel.  This is a continuous filter,
            // not a re-init-able trajectory segment — stream_dq /
            // stream_ddq carry across all cycles and across multiple
            // Python commands.  That continuity is the whole point.
            for (int i = 0; i < 7; ++i) {
                double err = stream_target_q[i] - stream_q[i];
                double desired_acc = kKp * err - kKd * stream_dq[i];
                desired_acc = std::clamp(desired_acc, -kMaxAcc, kMaxAcc);

                double max_da = kMaxJerk * dt;
                double da = std::clamp(desired_acc - stream_ddq[i],
                                       -max_da, max_da);
                stream_ddq[i] += da;

                stream_dq[i] += stream_ddq[i] * dt;
                stream_dq[i] = std::clamp(stream_dq[i], -kMaxVel, kMaxVel);

                stream_q[i] += stream_dq[i] * dt;
            }

            cmd_q = stream_q;
        }

        // Hard joint position limit + per-cycle vel cap.  If this
        // modifies cmd_q we MUST sync the OTG (and any ongoing move
        // segment's notion of "current" state) back to the actual
        // commanded value — otherwise stream_q drifts away from what
        // the robot was told, the integrator builds up phantom error,
        // and the next cycles try to "catch up" by accelerating into
        // the wall.  Zeroing dq/ddq here re-starts the OTG from rest at
        // the clamped position, which is the conservative choice (the
        // alternative — recomputing dq from finite difference — risks
        // re-triggering the same clamp).
        const bool clamped = ClampJointPosition(cmd_q, last_sent_q);
        if (clamped) {
            stream_q  = cmd_q;
            stream_dq = {0, 0, 0, 0, 0, 0, 0};
            stream_ddq = {0, 0, 0, 0, 0, 0, 0};
            // If a move was in progress, the trajectory's internal
            // tau-parameterisation will keep producing values that may
            // again hit the clamp; cancel it so the operator gets a
            // clean is_moving()=false signal and can decide what to do.
            if (rt_state == RTState::MOVING) {
                logger().warn(
                    "moveToJoints clamped at cycle %lu — cancelling move",
                    (unsigned long)iter);
                move_state.active = false;
                rt_state = RTState::STREAMING;
                moving_.store(false);
                stream_target_q = cmd_q;
            }
        }

        if (!CheckFinite(cmd_q)) {
            logger().error("Non-finite joint command, e-stop");
            estop_.store(true);
            return franka::MotionFinished(franka::JointPositions(last_sent_q));
        }

        last_sent_q = cmd_q;
        return franka::JointPositions(cmd_q);
    };

    try {
        robot_.control(callback,
                       franka::ControllerMode::kJointImpedance,
                       /*limit_rate=*/true);
    } catch (const franka::Exception& e) {
        logger().error("JointPositionControl robot.control exception (iter %lu): %s",
                       (unsigned long)iter, e.what());
        logger().error("  Last 5 cycle periods (oldest first):");
        for (int i = 0; i < 5; ++i) {
            int idx = (diag_idx + i) % 5;
            const auto& d = diag[idx];
            logger().error("    iter=%lu period=%.2fms",
                          (unsigned long)d.iter, d.period_ms);
        }
    } catch (...) {
        logger().error("JointPositionControl unknown exception (iter %lu)",
                       (unsigned long)iter);
    }

    running_.store(false);
    moving_.store(false);
}

}  // namespace franka_rt
