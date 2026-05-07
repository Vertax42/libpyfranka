#include "realtime_control/joint_position_control.hpp"

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"

namespace franka_rt {

JointPositionControl::JointPositionControl(franka::Robot& robot,
                                           double max_joint_step_per_cycle)
    : robot_(robot), max_step_(max_joint_step_per_cycle) {}

JointPositionControl::~JointPositionControl() {
    stop();
}

void JointPositionControl::start() {
    if (running_.exchange(true)) return;
    stop_requested_.store(false);
    estop_.store(false);
    moving_.store(false);

    rt_thread_ = std::thread(&JointPositionControl::rtLoop, this);
}

void JointPositionControl::stop() {
    if (!running_.load()) return;
    stop_requested_.store(true);
    if (rt_thread_.joinable()) rt_thread_.join();
    running_.store(false);
}

void JointPositionControl::setTargetJoints(const std::array<double, 7>& q) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    target_q_ = q;
    target_set_ = true;
    // External streaming command cancels any in-progress non-blocking move.
    traj_.cancel();
    moving_.store(false);
}

JointState JointPositionControl::getState() const noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    return latest_state_;
}

void JointPositionControl::moveToJoints(const std::array<double, 7>& q,
                                        double duration_sec) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    pending_traj_target_ = q;
    pending_traj_duration_ = duration_sec;
    traj_pending_init_ = true;
    moving_.store(true);
}

void JointPositionControl::cancelMove() noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    traj_.cancel();
    traj_pending_init_ = false;
    moving_.store(false);
}

void JointPositionControl::rtLoop() {
    // Try to elevate this thread to RT priority.  If the user did not configure
    // PAM rt limits or run as root, this fails — libfranka will still run, but
    // overruns become more likely.  We log a warning rather than fail.
    std::string err;
    if (!franka::setCurrentThreadToHighestSchedulerPriority(&err)) {
        logger().warn("Could not set RT priority on JointPosition thread: %s",
                      err.c_str());
    }

    try {
        ac_ = robot_.startJointPositionControl(
            research_interface::robot::Move::ControllerMode::kJointImpedance);
    } catch (const franka::Exception& e) {
        logger().error("startJointPositionControl failed: %s", e.what());
        running_.store(false);
        return;
    }

    std::array<double, 7> last_cmd{};
    bool first_cycle = true;

    try {
        while (!stop_requested_.load()) {
            auto [state, period] = ac_->readOnce();

            // 1. Snapshot state for Python
            JointState js;
            js.q = state.q;
            js.dq = state.dq;
            js.tau_J = state.tau_J;
            js.tau_ext_hat_filtered = state.tau_ext_hat_filtered;
            js.O_T_EE = state.O_T_EE;
            js.control_command_success_rate = state.control_command_success_rate;

            // 2. Determine command for this cycle
            std::array<double, 7> cmd_q{};
            bool have_cmd = false;
            {
                std::lock_guard<std::mutex> g(mtx_);
                latest_state_ = js;

                // First cycle: seed last_cmd with current measured q so we
                // ramp from where we are.
                if (first_cycle) {
                    last_cmd = state.q;
                    first_cycle = false;
                }

                // Initialize a pending trajectory if requested
                if (traj_pending_init_) {
                    traj_.init(last_cmd, pending_traj_target_, pending_traj_duration_);
                    traj_pending_init_ = false;
                }

                // Trajectory takes precedence over streaming target
                if (traj_.isActive()) {
                    bool cont = traj_.step(cmd_q);
                    have_cmd = true;
                    if (!cont) {
                        moving_.store(false);
                    }
                } else if (target_set_) {
                    cmd_q = target_q_;
                    have_cmd = true;
                }
            }

            if (estop_.load()) break;

            if (!have_cmd) {
                // No target yet — hold position
                cmd_q = state.q;
            }

            // 3. Per-cycle clamp toward target (rate limit)
            for (int i = 0; i < 7; ++i) {
                double delta = cmd_q[i] - last_cmd[i];
                if (std::abs(delta) > max_step_) {
                    cmd_q[i] = last_cmd[i] + std::copysign(max_step_, delta);
                }
            }

            // 4. Hard joint position limit
            ClampJointPosition(cmd_q, last_cmd);

            // 5. Sanity
            if (!CheckFinite(cmd_q)) {
                logger().error("Non-finite joint command, e-stopping");
                break;
            }

            // libfranka's active control performs internal rate limiting and
            // safety filtering — no need to call franka::limitRate() here.
            ac_->writeOnce(franka::JointPositions(cmd_q));
            last_cmd = cmd_q;
        }
    } catch (const franka::Exception& e) {
        logger().error("JointPosition RT loop exception: %s", e.what());
    }

    // Cleanly finish: write a final MotionFinished so libfranka exits the loop
    try {
        franka::JointPositions stop_cmd(last_cmd);
        ac_->writeOnce(franka::MotionFinished(stop_cmd));
    } catch (...) {
        // Already in error state — ignore
    }

    running_.store(false);
}

}  // namespace franka_rt
