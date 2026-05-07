#include "realtime_control/cartesian_pose_control.hpp"

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>

#include <cmath>

#include "realtime_control/logging.hpp"
#include "realtime_control/pose_math.hpp"
#include "realtime_control/rt_common.hpp"
#include "realtime_control/trajectory.hpp"

namespace franka_rt {

CartesianPoseControl::CartesianPoseControl(franka::Robot& robot,
                                           double max_translational_step,
                                           double max_rotational_step)
    : robot_(robot),
      max_t_step_(max_translational_step),
      max_r_step_(max_rotational_step) {}

CartesianPoseControl::~CartesianPoseControl() { stop(); }

void CartesianPoseControl::start() {
    if (running_.exchange(true)) return;
    stop_requested_.store(false);
    estop_.store(false);
    moving_.store(false);
    rt_thread_ = std::thread(&CartesianPoseControl::rtLoop, this);
}

void CartesianPoseControl::stop() {
    if (!running_.load()) return;
    stop_requested_.store(true);
    if (rt_thread_.joinable()) rt_thread_.join();
    running_.store(false);
}

void CartesianPoseControl::setTargetPose(const std::array<double, 16>& T) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    target_pose_ = T;
    target_set_ = true;
    traj_.cancel();
    moving_.store(false);
}

CartesianState CartesianPoseControl::getState() const noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    return latest_state_;
}

void CartesianPoseControl::moveToPose(const std::array<double, 16>& T,
                                      double duration_sec) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    pending_traj_target_ = T;
    pending_traj_duration_ = duration_sec;
    traj_pending_init_ = true;
    moving_.store(true);
}

void CartesianPoseControl::cancelMove() noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    traj_.cancel();
    traj_pending_init_ = false;
    moving_.store(false);
}

void CartesianPoseControl::rtLoop() {
    std::string err;
    if (!franka::setCurrentThreadToHighestSchedulerPriority(&err)) {
        logger().warn("Could not set RT priority on CartesianPose thread: %s",
                      err.c_str());
    }

    try {
        ac_ = robot_.startCartesianPoseControl(
            research_interface::robot::Move::ControllerMode::kCartesianImpedance);
    } catch (const franka::Exception& e) {
        logger().error("startCartesianPoseControl failed: %s", e.what());
        running_.store(false);
        return;
    }

    std::array<double, 16> last_cmd{};
    bool first_cycle = true;

    try {
        while (!stop_requested_.load()) {
            auto [state, period] = ac_->readOnce();

            CartesianState cs;
            cs.O_T_EE = state.O_T_EE;
            cs.O_dP_EE_d = state.O_dP_EE_d;
            cs.K_F_ext_hat_K = state.K_F_ext_hat_K;
            cs.O_F_ext_hat_K = state.O_F_ext_hat_K;
            cs.q = state.q;
            cs.dq = state.dq;
            cs.tau_ext_hat_filtered = state.tau_ext_hat_filtered;
            cs.control_command_success_rate = state.control_command_success_rate;

            std::array<double, 16> cmd{};
            bool have_cmd = false;
            {
                std::lock_guard<std::mutex> g(mtx_);
                latest_state_ = cs;

                if (first_cycle) {
                    last_cmd = state.O_T_EE;
                    first_cycle = false;
                }

                if (traj_pending_init_) {
                    traj_.init(last_cmd, pending_traj_target_, pending_traj_duration_);
                    traj_pending_init_ = false;
                }

                if (traj_.isActive()) {
                    bool cont = traj_.step(cmd);
                    have_cmd = true;
                    if (!cont) moving_.store(false);
                } else if (target_set_) {
                    cmd = target_pose_;
                    have_cmd = true;
                }
            }

            if (estop_.load()) break;
            if (!have_cmd) cmd = state.O_T_EE;

            if (!IsValidTransform(cmd)) {
                logger().error("Invalid 4x4 pose command, e-stopping");
                break;
            }

            // Per-cycle clamp: bound translation magnitude and rotation angle
            // step from last commanded pose.
            double dx = cmd[12] - last_cmd[12];
            double dy = cmd[13] - last_cmd[13];
            double dz = cmd[14] - last_cmd[14];
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > max_t_step_) {
                double s = max_t_step_ / dist;
                cmd[12] = last_cmd[12] + dx * s;
                cmd[13] = last_cmd[13] + dy * s;
                cmd[14] = last_cmd[14] + dz * s;
            }
            // (Rotation rate limiting is left to libfranka's internal limiter.)

            ac_->writeOnce(franka::CartesianPose(cmd));
            last_cmd = cmd;
        }
    } catch (const franka::Exception& e) {
        logger().error("CartesianPose RT loop exception: %s", e.what());
    }

    try {
        ac_->writeOnce(franka::MotionFinished(franka::CartesianPose(last_cmd)));
    } catch (...) {}

    running_.store(false);
}

}  // namespace franka_rt
