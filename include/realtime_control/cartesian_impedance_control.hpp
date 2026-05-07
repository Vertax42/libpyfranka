#pragma once
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include <franka/active_control_base.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "realtime_control/cartesian_state.hpp"

namespace franka_rt {

/// 1 kHz Cartesian impedance controller.
///
/// Python streams a target 4×4 pose plus optional 6D stiffness and damping.
/// At 1 kHz the C++ thread computes:
///     tau = J^T (K * pose_error - D * end_effector_twist) + coriolis
/// and sends it via libfranka torque control.  Gravity is auto-compensated by
/// the robot.
class CartesianImpedanceControl {
public:
    explicit CartesianImpedanceControl(franka::Robot& robot);
    ~CartesianImpedanceControl();

    CartesianImpedanceControl(const CartesianImpedanceControl&) = delete;
    CartesianImpedanceControl& operator=(const CartesianImpedanceControl&) = delete;

    void start();
    void stop();
    bool isRunning() const noexcept { return running_.load(); }
    void triggerEstop() noexcept { estop_.store(true); }

    void setTargetPose(const std::array<double, 16>& T) noexcept;
    void setStiffness(const std::array<double, 6>& K) noexcept;
    void setDamping(const std::array<double, 6>& D) noexcept;
    CartesianState getState() const noexcept;

private:
    void rtLoop();

    franka::Robot& robot_;
    std::unique_ptr<franka::Model> model_;
    std::unique_ptr<franka::ActiveControlBase> ac_;
    std::thread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> estop_{false};

    mutable std::mutex mtx_;
    std::array<double, 16> target_pose_{};
    bool target_set_ = false;
    // Default: 300 N/m translational, 30 Nm/rad rotational, critical damping.
    std::array<double, 6> stiffness_{300, 300, 300, 30, 30, 30};
    std::array<double, 6> damping_{
        2.0 * std::sqrt(300.0), 2.0 * std::sqrt(300.0), 2.0 * std::sqrt(300.0),
        2.0 * std::sqrt(30.0),  2.0 * std::sqrt(30.0),  2.0 * std::sqrt(30.0)};
    CartesianState latest_state_{};
};

}  // namespace franka_rt
