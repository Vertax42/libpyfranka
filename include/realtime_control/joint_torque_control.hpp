#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <franka/active_control_base.h>
#include <franka/robot.h>

#include "realtime_control/joint_state.hpp"

namespace franka_rt {

/// 1 kHz joint-torque streaming controller.  Python is responsible for the
/// control law (e.g. their own joint impedance / friction compensation).
/// Torques sent here are added on top of libfranka's gravity compensation.
class JointTorqueControl {
public:
    explicit JointTorqueControl(franka::Robot& robot);
    ~JointTorqueControl();

    JointTorqueControl(const JointTorqueControl&) = delete;
    JointTorqueControl& operator=(const JointTorqueControl&) = delete;

    void start();
    void stop();
    bool isRunning() const noexcept { return running_.load(); }
    void triggerEstop() noexcept { estop_.store(true); }

    void setTargetTorques(const std::array<double, 7>& tau) noexcept;
    JointState getState() const noexcept;

private:
    void rtLoop();

    franka::Robot& robot_;
    std::unique_ptr<franka::ActiveControlBase> ac_;
    std::thread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> estop_{false};

    mutable std::mutex mtx_;
    std::array<double, 7> target_tau_{};
    bool target_set_ = false;
    JointState latest_state_{};
};

}  // namespace franka_rt
