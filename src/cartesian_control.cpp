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

double Norm3(const std::array<double, 3>& v) noexcept {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

bool ClampNorm3(std::array<double, 3>& v, double max_norm) noexcept {
    if (max_norm <= 0.0) return false;
    const double n = Norm3(v);
    if (n <= max_norm || n <= 1e-12) return false;
    const double scale = max_norm / n;
    v[0] *= scale;
    v[1] *= scale;
    v[2] *= scale;
    return true;
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

    // Rotation OTG state — full PD + jerk-limit, structurally identical
    // to the translation OTG.  Each cycle:
    //   axis-angle err in BASE frame → PD → desired α (clamped) →
    //   jerk-limit Δα → integrate ω → integrate quat → renormalise.
    //
    //   stream_quat:  current commanded quaternion (wxyz, persistent C2)
    //   stream_omega: angular velocity in BASE frame (rad/s)
    //   stream_alpha: angular acceleration in BASE frame (rad/s²)
    //
    // Why a 1st-order SLERP filter was insufficient: with α = dt/(τ+dt)
    // a target rotation step Δθ produces an immediate output angular
    // rate of (1/τ)·Δθ — appearing in ONE cycle.  At τ=30 ms and Δθ=10
    // mrad that is ~0.3 rad/s, i.e. an angular acceleration of
    // ~300 rad/s² in a single cycle, far above libfranka's ~25 rad/s²
    // threshold.  Spacemouse 30 Hz writes deliver exactly this pattern
    // of small per-write rotation steps and triggered
    // `cartesian_motion_generator_joint_acceleration_discontinuity`
    // ~8 s into active teleop.  PD + jerk-limit smooths the rate ramp
    // so neither velocity nor acceleration has a step at target jumps.
    std::array<double, 4>  stream_quat{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3>  stream_omega{};
    std::array<double, 3>  stream_alpha{};

    // Persistent streaming target — updated only when a new Python command
    // arrives.  In STREAMING mode we do not forward this sparse 30 Hz target
    // directly.  Instead a 1 kHz jerk-limited online trajectory generator
    // advances `stream_pose` toward it.  This preserves the CartesianPose
    // motion-generator semantics while matching the official successful
    // callback tests: every callback returns a smooth pose sample.
    std::array<double, 16> stream_target_pose{};

    MinJerkTrajectory move_traj;
    std::array<double, 16> queued_move_target{};
    double queued_move_duration = 0.0;

    std::chrono::steady_clock::time_point last_cmd_time;
    bool   last_cmd_time_valid = false;
    double cmd_interval_sec    = kDefaultCommandInterval;

    enum class RTState { STREAMING, STOPPING_FOR_MOVE, MOVING };
    RTState rt_state = RTState::STREAMING;

    // Diagnostic ring buffer — for crash post-mortems.  We only write a
    // fixed-size struct in the RT loop and print it after control() unwinds,
    // so normal 1 kHz operation never pays logging/I/O cost.
    struct DiagSample {
        uint64_t iter = 0;
        double   period_ms = 0.0;
        double   cmd_success = 0.0;
        uint64_t errors = 0;
        int      rt_state = 0;       // 0=streaming, 1=moving, 2=stopping_for_move
        bool     got_cmd = false;
        bool     cmd_accepted = false;
        bool     cmd_dropped_jump = false;
        bool     cmd_dropped_bad = false;
        double   cmd_interval_ms = 0.0;
        double   target_pos_err_mm = 0.0;
        double   target_rot_err_mrad = 0.0;
        double   out_step_pos_um = 0.0;
        double   out_step_rot_urad = 0.0;
        double   stream_vel_norm = 0.0;
        double   stream_acc_norm = 0.0;
        double   stream_omega_norm = 0.0;
        double   stream_alpha_norm = 0.0;
        double   measured_cmd_pos_mm = 0.0;
        double   measured_cmd_rot_mrad = 0.0;
    };
    constexpr int kDiagSize = 32;
    std::array<DiagSample, kDiagSize> diag{};
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
            // Seed rotation OTG state from libfranka's commanded pose so
            // the first incoming Python target's tiny round-trip rotation
            // delta drives a smooth SLERP rather than an instantaneous
            // matrix-block copy.
            std::array<double, 3> _p_seed;
            detail::mat4ToPosQuat(state.O_T_EE_c, _p_seed, stream_quat);
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

        // ---- Diagnostic record for this cycle ----
        DiagSample cur_diag;
        cur_diag.iter = iter;
        cur_diag.period_ms = period.toSec() * 1000.0;
        cur_diag.cmd_success = state.control_command_success_rate;
        cur_diag.errors = PackErrors(state.current_errors);
        cur_diag.rt_state = (rt_state == RTState::MOVING)
            ? 1
            : ((rt_state == RTState::STOPPING_FOR_MOVE) ? 2 : 0);
        cur_diag.measured_cmd_pos_mm =
            PoseTranslationDist(state.O_T_EE, state.O_T_EE_c) * 1000.0;
        cur_diag.measured_cmd_rot_mrad =
            QuatAngularDist(state.O_T_EE, state.O_T_EE_c) * 1000.0;
        auto record_diag = [&]() noexcept {
            diag[diag_idx] = cur_diag;
            diag_idx = (diag_idx + 1) % kDiagSize;
        };
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
            record_diag();
            return franka::MotionFinished(franka::CartesianPose(last_sent_pose));
        }
        if (state.current_errors) {
            record_diag();
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
                    // Re-seed rotation OTG from the held pose; clear
                    // angular vel / accel so streaming resumes from
                    // rest rather than carrying stale rates through.
                    std::array<double, 3> _p_unused;
                    detail::mat4ToPosQuat(last_sent_pose, _p_unused, stream_quat);
                    stream_omega = {0.0, 0.0, 0.0};
                    stream_alpha = {0.0, 0.0, 0.0};
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

            // Graceful stop must also decelerate orientation.  If we stop
            // sending the angular OTG output immediately while
            // stream_omega is non-zero, libfranka sees a commanded angular
            // velocity step at MotionFinished time and can fault with
            // cartesian/joint velocity discontinuities during Ctrl+C.
            constexpr double kRotStopKd = 12.0;
            constexpr double kRotStopMaxAcc = 2.0;    // rad/s^2, vector norm
            constexpr double kRotStopMaxJerk = 20.0;  // rad/s^3, vector norm
            std::array<double, 3> desired_alpha{};
            for (int i = 0; i < 3; ++i) {
                desired_alpha[i] = -kRotStopKd * stream_omega[i];
            }
            ClampNorm3(desired_alpha, kRotStopMaxAcc);

            std::array<double, 3> delta_alpha = {
                desired_alpha[0] - stream_alpha[0],
                desired_alpha[1] - stream_alpha[1],
                desired_alpha[2] - stream_alpha[2],
            };
            ClampNorm3(delta_alpha, kRotStopMaxJerk * dt);
            for (int i = 0; i < 3; ++i) {
                stream_alpha[i] += delta_alpha[i];
            }
            ClampNorm3(stream_alpha, kRotStopMaxAcc);

            for (int i = 0; i < 3; ++i) {
                stream_omega[i] += stream_alpha[i] * dt;
            }
            ClampNorm3(stream_omega, 0.8);

            if (Norm3(stream_omega) < 1e-4 && Norm3(stream_alpha) < 1e-3) {
                stream_omega = {0.0, 0.0, 0.0};
                stream_alpha = {0.0, 0.0, 0.0};
            } else {
                quiet = false;
            }

            const double qcw = stream_quat[0], qcx = stream_quat[1],
                         qcy = stream_quat[2], qcz = stream_quat[3];
            const double wx = stream_omega[0],
                         wy = stream_omega[1],
                         wz = stream_omega[2];
            const double half_dt = 0.5 * dt;
            stream_quat[0] += -half_dt * (wx*qcx + wy*qcy + wz*qcz);
            stream_quat[1] +=  half_dt * (qcw*wx + (wy*qcz - wz*qcy));
            stream_quat[2] +=  half_dt * (qcw*wy + (wz*qcx - wx*qcz));
            stream_quat[3] +=  half_dt * (qcw*wz + (wx*qcy - wy*qcx));

            const double qn2 = stream_quat[0]*stream_quat[0]
                             + stream_quat[1]*stream_quat[1]
                             + stream_quat[2]*stream_quat[2]
                             + stream_quat[3]*stream_quat[3];
            if (qn2 > 1e-24) {
                const double inv = 1.0 / std::sqrt(qn2);
                stream_quat[0] *= inv;
                stream_quat[1] *= inv;
                stream_quat[2] *= inv;
                stream_quat[3] *= inv;
            }
            std::array<double, 3> p_keep = {
                stream_pose[12], stream_pose[13], stream_pose[14]
            };
            detail::posQuatToMat4(p_keep, stream_quat, stream_pose);

            last_sent_pose = stream_pose;
            cur_diag.stream_vel_norm = std::sqrt(stream_vel[0]*stream_vel[0] +
                                                 stream_vel[1]*stream_vel[1] +
                                                 stream_vel[2]*stream_vel[2]);
            cur_diag.stream_acc_norm = std::sqrt(stream_acc[0]*stream_acc[0] +
                                                 stream_acc[1]*stream_acc[1] +
                                                 stream_acc[2]*stream_acc[2]);
            cur_diag.stream_omega_norm = Norm3(stream_omega);
            cur_diag.stream_alpha_norm = Norm3(stream_alpha);
            cur_diag.out_step_pos_um =
                PoseTranslationDist(stream_pose, state.O_T_EE_c) * 1e6;
            cur_diag.out_step_rot_urad =
                QuatAngularDist(stream_pose, state.O_T_EE_c) * 1e6;
            record_diag();
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
        cur_diag.got_cmd = got_cmd;
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
            queued_move_target = move_target;
            queued_move_duration = move_duration;
            stream_target_pose = last_sent_pose;
            rt_state = RTState::STOPPING_FOR_MOVE;
            moving_.store(true);
            logger().info("moveToPose requested while streaming; decelerating before reset trajectory");
        }

        std::array<double, 16> T_cmd = stream_target_pose;

        if (rt_state == RTState::MOVING) {
            bool in_progress = move_traj.step(dt, T_cmd);
                if (!in_progress) {
                    rt_state = RTState::STREAMING;
                    moving_.store(false);
                    stream_target_pose = T_cmd;
                    stream_pose = T_cmd;
                    stream_vel = {0.0, 0.0, 0.0};
                    stream_acc = {0.0, 0.0, 0.0};
                    // Re-seed rotation OTG from the move endpoint and
                    // clear angular state so streaming resumes at rest.
                    std::array<double, 3> _p_unused;
                    detail::mat4ToPosQuat(T_cmd, _p_unused, stream_quat);
                    stream_omega = {0.0, 0.0, 0.0};
                    stream_alpha = {0.0, 0.0, 0.0};
                }
                cur_diag.stream_vel_norm =
                    PoseTranslationDist(T_cmd, last_sent_pose) / dt;
                cur_diag.stream_omega_norm =
                    QuatAngularDist(T_cmd, last_sent_pose) / dt;
        } else {
            // STREAMING.
            if (rt_state == RTState::STREAMING && got_cmd) {
                if (!CheckFinite(latest_cmd.T)) {
                    cur_diag.cmd_dropped_bad = true;
                    logger().error("NaN/Inf in command pose; e-stop");
                    estop_.store(true);
                    record_diag();
                    return franka::MotionFinished(franka::CartesianPose(last_sent_pose));
                }
                if (!franka::isHomogeneousTransformation(latest_cmd.T)) {
                    cur_diag.cmd_dropped_bad = true;
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
                    cur_diag.cmd_interval_ms = cmd_interval_sec * 1000.0;

                    double pos_thresh = kMaxPositionVelocity * cmd_interval_sec;
                    double rot_thresh = kMaxRotationVelocity * cmd_interval_sec;
                    if (CheckCartesianJump(latest_cmd.T, stream_target_pose,
                                           pos_thresh, rot_thresh)) {
                        cur_diag.cmd_dropped_jump = true;
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
                        cur_diag.cmd_accepted = true;
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
            cur_diag.stream_vel_norm = std::sqrt(stream_vel[0]*stream_vel[0] +
                                                 stream_vel[1]*stream_vel[1] +
                                                 stream_vel[2]*stream_vel[2]);
            cur_diag.stream_acc_norm = std::sqrt(stream_acc[0]*stream_acc[0] +
                                                 stream_acc[1]*stream_acc[1] +
                                                 stream_acc[2]*stream_acc[2]);

            // Rotation OTG — PD + jerk-limit, structurally identical to
            // the translation OTG above.  Operates on:
            //   stream_quat  : current commanded quaternion (wxyz)
            //   stream_omega : angular velocity in BASE frame (rad/s)
            //   stream_alpha : angular acceleration in BASE frame (rad/s²)
            //
            // Step:
            //   1. Extract target quaternion, take short-arc.
            //   2. Compute axis-angle error from stream_quat to q_target
            //      in BASE frame.  err_aa = 2 * vector(q_err) when
            //      |q_err.w| ≈ 1, with an axis*angle reconstruction
            //      otherwise.
            //   3. desired_alpha = kRotKp · err_aa − kRotKd · stream_omega,
            //      clamp by kMaxAngAcc.
            //   4. Δα limited by kMaxAngJerk · dt; integrate stream_alpha.
            //   5. Integrate stream_omega += stream_alpha · dt; clamp by
            //      kMaxAngVel.
            //   6. Integrate quaternion: q̇ = ½ · ωquat ⊗ q ; q += q̇·dt.
            //   7. Renormalise stream_quat (essential — without it
            //      cumulative error makes posQuatToMat4 produce
            //      non-orthogonal rotation matrices that libfranka
            //      silently rejects).
            //
            // Gains chosen conservatively for 7-DOF IK.  The limits below
            // are vector-norm limits, not per-axis limits.  The previous
            // per-axis clamp allowed |omega| to reach sqrt(3) * limit; logs
            // showed |omega|≈2.25 rad/s and |alpha|≈6.25 rad/s^2 just before
            // `cartesian_motion_generator_joint_acceleration_discontinuity`.
            // Keep these low first; raise them only after sustained teleop
            // passes without reflex.
            constexpr double kRotKp     = 36.0;   // 1/s²
            constexpr double kRotKd     = 12.0;   // 1/s
            constexpr double kMaxAngVel = 0.8;    // rad/s, vector norm
            constexpr double kMaxAngAcc = 2.0;    // rad/s², vector norm
            constexpr double kMaxAngJerk = 20.0;  // rad/s³, vector norm

            // (1) Extract target quaternion, short-arc.
            std::array<double, 3> _p_unused;
            std::array<double, 4> q_target;
            detail::mat4ToPosQuat(stream_target_pose, _p_unused, q_target);
            if (detail::quatDot(stream_quat, q_target) < 0.0) {
                for (int k = 0; k < 4; ++k) q_target[k] = -q_target[k];
            }

            // (2) Axis-angle error in BASE frame: err_q = q_target * conj(q_curr).
            //     The vector part of err_q is (1/2) * sin(θ/2) * axis;
            //     for small θ this is (1/2) * (θ/2) * axis ≈ (θ/4) * axis.
            //     Multiplying the vector part by 2 / max(w, ε) recovers
            //     axis * tan(θ/2), and for θ ≪ π that ≈ axis * (θ/2);
            //     multiply by 2 once more → axis * θ.  We use the
            //     exact form with atan2 to stay correct at large θ.
            const double qcw = stream_quat[0], qcx = stream_quat[1],
                         qcy = stream_quat[2], qcz = stream_quat[3];
            const double qtw = q_target[0],   qtx = q_target[1],
                         qty = q_target[2],   qtz = q_target[3];
            // err_q = q_target * conj(q_curr); conj(q_curr) has
            // vector part (-qcx,-qcy,-qcz).  Be careful with the cross
            // product sign here:
            //   vt x (-vc) = -(vt x vc)
            // A previous version had the opposite sign for this term,
            // which made the angular OTG diverge from the target in
            // some orientations (target_rot grew while SpaceMouse output
            // was zero).
            const double ew =  qtw*qcw + qtx*qcx + qty*qcy + qtz*qcz;
            const double ex =  qtx*qcw - qtw*qcx - qty*qcz + qtz*qcy;
            const double ey =  qty*qcw - qtw*qcy - qtz*qcx + qtx*qcz;
            const double ez =  qtz*qcw - qtw*qcz - qtx*qcy + qty*qcx;
            std::array<double, 3> err_aa{};
            const double sinhalf = std::sqrt(ex*ex + ey*ey + ez*ez);
            if (sinhalf < 1e-12) {
                // Already aligned; err = 0.
                err_aa = {0.0, 0.0, 0.0};
            } else {
                const double angle = 2.0 * std::atan2(sinhalf, ew);
                const double scale = angle / sinhalf;
                err_aa = {ex * scale, ey * scale, ez * scale};
            }

            // (3) PD target angular acceleration, clamped by vector norm.
            std::array<double, 3> desired_alpha{};
            for (int i = 0; i < 3; ++i) {
                desired_alpha[i] = kRotKp * err_aa[i] - kRotKd * stream_omega[i];
            }
            ClampNorm3(desired_alpha, kMaxAngAcc);

            // (4) Jerk-limit the angular acceleration vector.
            std::array<double, 3> delta_alpha = {
                desired_alpha[0] - stream_alpha[0],
                desired_alpha[1] - stream_alpha[1],
                desired_alpha[2] - stream_alpha[2],
            };
            ClampNorm3(delta_alpha, kMaxAngJerk * dt);
            for (int i = 0; i < 3; ++i) {
                stream_alpha[i] += delta_alpha[i];
            }
            ClampNorm3(stream_alpha, kMaxAngAcc);

            // (5) Integrate angular velocity and clamp by vector norm.
            for (int i = 0; i < 3; ++i) {
                stream_omega[i] += stream_alpha[i] * dt;
            }
            ClampNorm3(stream_omega, kMaxAngVel);
            cur_diag.stream_omega_norm =
                Norm3(stream_omega);
            cur_diag.stream_alpha_norm =
                Norm3(stream_alpha);

            // (6) Integrate quaternion: q̇ = ½ · (0, ω) ⊗ q (BASE frame).
            //   q̇.w = -½ (ω·v)        where v = (qx,qy,qz)
            //   q̇.v =  ½ (qw·ω + ω×v)
            const double wx = stream_omega[0],
                         wy = stream_omega[1],
                         wz = stream_omega[2];
            const double half_dt = 0.5 * dt;
            const double dqw = -half_dt * (wx*qcx + wy*qcy + wz*qcz);
            const double dqx =  half_dt * (qcw*wx + (wy*qcz - wz*qcy));
            const double dqy =  half_dt * (qcw*wy + (wz*qcx - wx*qcz));
            const double dqz =  half_dt * (qcw*wz + (wx*qcy - wy*qcx));
            stream_quat[0] += dqw;
            stream_quat[1] += dqx;
            stream_quat[2] += dqy;
            stream_quat[3] += dqz;

            // (7) Renormalise — without this, after thousands of cycles
            //     the quat magnitude drifts away from 1 and posQuatToMat4
            //     produces a non-orthogonal rotation matrix that
            //     libfranka silently rejects (manifests as cmd_succ
            //     gradually dropping below 1.0 and eventually a
            //     joint-side discontinuity reflex).
            const double n2 = stream_quat[0]*stream_quat[0]
                            + stream_quat[1]*stream_quat[1]
                            + stream_quat[2]*stream_quat[2]
                            + stream_quat[3]*stream_quat[3];
            if (n2 > 1e-24) {
                const double inv = 1.0 / std::sqrt(n2);
                stream_quat[0] *= inv;
                stream_quat[1] *= inv;
                stream_quat[2] *= inv;
                stream_quat[3] *= inv;
            }

            // Write smoothed rotation back into stream_pose, preserving
            // the translation we OTG-integrated above.
            std::array<double, 3> p_keep = {
                stream_pose[12], stream_pose[13], stream_pose[14]
            };
            detail::posQuatToMat4(p_keep, stream_quat, stream_pose);

            T_cmd = stream_pose;

            if (rt_state == RTState::STOPPING_FOR_MOVE) {
                constexpr double kMoveStartMaxVel = 0.001;      // m/s
                constexpr double kMoveStartMaxAcc = 0.01;       // m/s^2
                constexpr double kMoveStartMaxOmega = 0.005;    // rad/s
                constexpr double kMoveStartMaxAlpha = 0.05;     // rad/s^2
                const bool quiet_for_move =
                    cur_diag.stream_vel_norm <= kMoveStartMaxVel &&
                    cur_diag.stream_acc_norm <= kMoveStartMaxAcc &&
                    cur_diag.stream_omega_norm <= kMoveStartMaxOmega &&
                    cur_diag.stream_alpha_norm <= kMoveStartMaxAlpha;
                if (quiet_for_move) {
                    stream_vel = {0.0, 0.0, 0.0};
                    stream_acc = {0.0, 0.0, 0.0};
                    stream_omega = {0.0, 0.0, 0.0};
                    stream_alpha = {0.0, 0.0, 0.0};
                    const double duration = queued_move_duration;
                    move_traj.init(T_cmd, queued_move_target, duration);
                    rt_state = RTState::MOVING;
                    logger().info(
                        "Starting moveToPose reset trajectory after smooth stop "
                        "(duration=%.3fs)",
                        duration);
                }
            }
        }

        cur_diag.target_pos_err_mm =
            PoseTranslationDist(stream_target_pose, stream_pose) * 1000.0;
        cur_diag.target_rot_err_mrad =
            QuatAngularDist(stream_target_pose, stream_pose) * 1000.0;
        cur_diag.out_step_pos_um =
            PoseTranslationDist(T_cmd, last_sent_pose) * 1e6;
        cur_diag.out_step_rot_urad =
            QuatAngularDist(T_cmd, last_sent_pose) * 1e6;
        record_diag();
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
        logger().error("  Last %d RT diagnostic samples (oldest first):", kDiagSize);
        logger().error(
            "    columns: iter period_ms succ err state got acc jump bad "
            "cmd_dt_ms tgt_pos_mm tgt_rot_mrad step_pos_um step_rot_urad "
            "|v| |a| |w| |alpha| meas_cmd_pos_mm meas_cmd_rot_mrad");
        for (int i = 0; i < kDiagSize; ++i) {
            int idx = (diag_idx + i) % kDiagSize;
            const auto& d = diag[idx];
            if (d.iter == 0 && d.period_ms == 0.0) continue;
            logger().error(
                "    iter=%lu period=%.2f succ=%.3f err=0x%llx state=%d "
                "got=%d acc=%d jump=%d bad=%d cmd_dt=%.2f "
                "tgt_pos=%.3f tgt_rot=%.3f step_pos=%.2f step_rot=%.2f "
                "v=%.4f a=%.4f w=%.4f alpha=%.4f meas_cmd_pos=%.3f meas_cmd_rot=%.3f",
                (unsigned long)d.iter,
                d.period_ms,
                d.cmd_success,
                (unsigned long long)d.errors,
                d.rt_state,
                d.got_cmd ? 1 : 0,
                d.cmd_accepted ? 1 : 0,
                d.cmd_dropped_jump ? 1 : 0,
                d.cmd_dropped_bad ? 1 : 0,
                d.cmd_interval_ms,
                d.target_pos_err_mm,
                d.target_rot_err_mrad,
                d.out_step_pos_um,
                d.out_step_rot_urad,
                d.stream_vel_norm,
                d.stream_acc_norm,
                d.stream_omega_norm,
                d.stream_alpha_norm,
                d.measured_cmd_pos_mm,
                d.measured_cmd_rot_mrad);
        }
    } catch (...) {
        logger().error("CartesianControl unknown exception (iter %lu)",
                       (unsigned long)iter);
    }

    running_.store(false);
    moving_.store(false);
}

}  // namespace franka_rt
