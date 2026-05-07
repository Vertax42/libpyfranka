#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/active_control_base.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "realtime_control/joint_state.hpp"
#include "realtime_control/trajectory.hpp"

namespace franka_rt {

/// 1 kHz joint-position streaming controller.
///
/// Python writes a 7D target via `setTargetJoints()`.  An RT thread reads
/// libfranka state via `ActiveControlBase::readOnce()`, linearly interpolates
/// from the last commanded position toward the latest target (per cycle), and
/// writes the result via `writeOnce(JointPositions{...})`.
///
/// libfranka runs an internal joint-impedance tracking controller behind the
/// streamed positions (`ControllerMode::kJointImpedance`).
class JointPositionControl {
public:
    JointPositionControl(franka::Robot& robot, double max_joint_step_per_cycle = 0.001);
    ~JointPositionControl();

    JointPositionControl(const JointPositionControl&) = delete;
    JointPositionControl& operator=(const JointPositionControl&) = delete;

    // ---- Lifecycle ----
    void start();                          // launch RT thread + libfranka active control
    void stop();                           // signal stop, join RT thread
    bool isRunning() const noexcept { return running_.load(); }
    void triggerEstop() noexcept { estop_.store(true); }

    // ---- Python-facing ----
    void setTargetJoints(const std::array<double, 7>& q) noexcept;
    JointState getState() const noexcept;

    // Non-blocking move from current position to target via min-jerk-ish
    // joint trajectory at 1 kHz.  Sets is_moving() until trajectory completes.
    void moveToJoints(const std::array<double, 7>& q, double duration_sec = 0.0) noexcept;
    bool isMoving() const noexcept { return moving_.load(); }
    void cancelMove() noexcept;

private:
    void rtLoop();

    franka::Robot& robot_;
    std::unique_ptr<franka::ActiveControlBase> ac_;
    std::thread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> estop_{false};
    std::atomic<bool> moving_{false};

    double max_step_;  // max per-cycle joint delta [rad/s × dt]

    // Shared state between Python and RT
    mutable std::mutex mtx_;
    std::array<double, 7> target_q_{};
    bool target_set_ = false;
    JointState latest_state_{};
    JointLinearTrajectory traj_{};
    bool traj_pending_init_ = false;
    std::array<double, 7> pending_traj_target_{};
    double pending_traj_duration_ = 0.0;
};

}  // namespace franka_rt
