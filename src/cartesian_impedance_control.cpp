#include "realtime_control/cartesian_impedance_control.hpp"

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>

#include <cmath>

#include "realtime_control/logging.hpp"
#include "realtime_control/pose_math.hpp"
#include "realtime_control/rt_common.hpp"

namespace franka_rt {

CartesianImpedanceControl::CartesianImpedanceControl(franka::Robot& robot)
    : robot_(robot) {}

CartesianImpedanceControl::~CartesianImpedanceControl() { stop(); }

void CartesianImpedanceControl::start() {
    if (running_.exchange(true)) return;
    stop_requested_.store(false);
    estop_.store(false);

    // loadModel() can block briefly — do it before launching the RT thread.
    try {
        model_ = std::make_unique<franka::Model>(robot_.loadModel());
    } catch (const franka::Exception& e) {
        logger().error("loadModel failed: %s", e.what());
        running_.store(false);
        return;
    }

    rt_thread_ = std::thread(&CartesianImpedanceControl::rtLoop, this);
}

void CartesianImpedanceControl::stop() {
    if (!running_.load()) return;
    stop_requested_.store(true);
    if (rt_thread_.joinable()) rt_thread_.join();
    running_.store(false);
}

void CartesianImpedanceControl::setTargetPose(const std::array<double, 16>& T) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    target_pose_ = T;
    target_set_ = true;
}

void CartesianImpedanceControl::setStiffness(const std::array<double, 6>& K) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    stiffness_ = K;
}

void CartesianImpedanceControl::setDamping(const std::array<double, 6>& D) noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    damping_ = D;
}

CartesianState CartesianImpedanceControl::getState() const noexcept {
    std::lock_guard<std::mutex> g(mtx_);
    return latest_state_;
}

void CartesianImpedanceControl::rtLoop() {
    std::string err;
    if (!franka::setCurrentThreadToHighestSchedulerPriority(&err)) {
        logger().warn("Could not set RT priority on CartesianImpedance thread: %s",
                      err.c_str());
    }

    try {
        ac_ = robot_.startTorqueControl();
    } catch (const franka::Exception& e) {
        logger().error("startTorqueControl failed: %s", e.what());
        running_.store(false);
        return;
    }

    bool first_cycle = true;
    std::array<double, 6> K{}, D{};
    std::array<double, 16> target{};
    bool have_target = false;

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

            {
                std::lock_guard<std::mutex> g(mtx_);
                latest_state_ = cs;
                K = stiffness_;
                D = damping_;
                if (target_set_) { target = target_pose_; have_target = true; }
                // First-time seed: hold current measured pose.
                if (!have_target && first_cycle) {
                    target = state.O_T_EE;
                    have_target = true;
                }
                if (first_cycle) first_cycle = false;
            }

            if (estop_.load()) break;
            if (!have_target) {
                // Send zero torque (gravity comp only)
                ac_->writeOnce(franka::Torques(std::array<double, 7>{}));
                continue;
            }

            // 1. Pose error (6D twist)
            auto err6 = PoseError(target, state.O_T_EE);

            // 2. Current end-effector twist via Jacobian
            auto J = model_->zeroJacobian(franka::Frame::kEndEffector, state);
            auto twist = JacobianTwist(J, state.dq);

            // 3. Cartesian wrench from PD on twist error
            std::array<double, 6> wrench;
            for (int i = 0; i < 6; ++i) {
                wrench[i] = K[i] * err6[i] - D[i] * twist[i];
            }

            // 4. Map wrench to joint torques
            auto tau_task = JacobianTransposeWrench(J, wrench);

            // 5. Add coriolis (gravity is added by libfranka internally)
            auto cor = model_->coriolis(state);
            std::array<double, 7> tau{};
            for (int i = 0; i < 7; ++i) {
                tau[i] = tau_task[i] + cor[i];
            }

            if (!CheckFinite(tau)) {
                logger().error("Non-finite impedance torque, e-stopping");
                break;
            }
            ClampJointTorque(tau);

            ac_->writeOnce(franka::Torques(tau));
        }
    } catch (const franka::Exception& e) {
        logger().error("CartesianImpedance RT loop exception: %s", e.what());
    }

    try {
        ac_->writeOnce(franka::MotionFinished(franka::Torques(std::array<double, 7>{})));
    } catch (...) {}

    running_.store(false);
}

}  // namespace franka_rt
