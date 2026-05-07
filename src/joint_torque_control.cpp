#include "realtime_control/joint_torque_control.hpp"

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>

#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"

namespace franka_rt {

JointTorqueControl::JointTorqueControl(franka::Robot& robot) : robot_(robot) {}

JointTorqueControl::~JointTorqueControl() { stop(); }

void JointTorqueControl::start() {
    if (running_.exchange(true)) return;
    stop_requested_.store(false);
    estop_.store(false);
    rt_thread_ = std::thread(&JointTorqueControl::rtLoop, this);
}

void JointTorqueControl::stop() {
    if (!running_.load()) return;
    stop_requested_.store(true);
    if (rt_thread_.joinable()) rt_thread_.join();
    running_.store(false);
}

void JointTorqueControl::setTargetTorques(const std::array<double, 7>& tau) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    target_tau_ = tau;
    target_set_ = true;
}

JointState JointTorqueControl::getState() const noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    return latest_state_;
}

void JointTorqueControl::rtLoop() {
    std::string err;
    if (!franka::setCurrentThreadToHighestSchedulerPriority(&err)) {
        logger().warn("Could not set RT priority on JointTorque thread: %s",
                      err.c_str());
    }

    try {
        ac_ = robot_.startTorqueControl();
    } catch (const franka::Exception& e) {
        logger().error("startTorqueControl failed: %s", e.what());
        running_.store(false);
        return;
    }

    std::array<double, 7> last_cmd{};

    try {
        while (!stop_requested_.load()) {
            auto [state, period] = ac_->readOnce();

            JointState js;
            js.q = state.q;
            js.dq = state.dq;
            js.tau_J = state.tau_J;
            js.tau_ext_hat_filtered = state.tau_ext_hat_filtered;
            js.O_T_EE = state.O_T_EE;
            js.control_command_success_rate = state.control_command_success_rate;

            std::array<double, 7> cmd{};
            bool have_cmd = false;
            {
                std::lock_guard<std::mutex> g(mtx_);
                latest_state_ = js;
                if (target_set_) { cmd = target_tau_; have_cmd = true; }
            }

            if (estop_.load()) break;
            if (!have_cmd) cmd = {};  // hold zero torque (libfranka handles gravity)

            if (!CheckFinite(cmd)) {
                logger().error("Non-finite torque command, e-stopping");
                break;
            }

            ClampJointTorque(cmd);
            ac_->writeOnce(franka::Torques(cmd));
            last_cmd = cmd;
        }
    } catch (const franka::Exception& e) {
        logger().error("JointTorque RT loop exception: %s", e.what());
    }

    try {
        ac_->writeOnce(franka::MotionFinished(franka::Torques(std::array<double, 7>{})));
    } catch (...) {}

    running_.store(false);
}

}  // namespace franka_rt
