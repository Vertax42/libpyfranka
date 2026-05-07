#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <franka/active_control_base.h>
#include <franka/robot.h>

#include "realtime_control/cartesian_state.hpp"
#include "realtime_control/trajectory.hpp"

namespace franka_rt {

/// 1 kHz Cartesian-pose streaming controller (libfranka internal Cartesian
/// impedance tracker handles the actual servoing).
class CartesianPoseControl {
public:
    CartesianPoseControl(franka::Robot& robot,
                         double max_translational_step = 0.001,
                         double max_rotational_step = 0.005);
    ~CartesianPoseControl();

    CartesianPoseControl(const CartesianPoseControl&) = delete;
    CartesianPoseControl& operator=(const CartesianPoseControl&) = delete;

    void start();
    void stop();
    bool isRunning() const noexcept { return running_.load(); }
    void triggerEstop() noexcept { estop_.store(true); }

    void setTargetPose(const std::array<double, 16>& O_T_EE) noexcept;
    CartesianState getState() const noexcept;

    void moveToPose(const std::array<double, 16>& O_T_EE,
                    double duration_sec = 0.0) noexcept;
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

    double max_t_step_;
    double max_r_step_;

    mutable std::mutex mtx_;
    std::array<double, 16> target_pose_{};
    bool target_set_ = false;
    CartesianState latest_state_{};
    LinearTrajectory traj_{};
    bool traj_pending_init_ = false;
    std::array<double, 16> pending_traj_target_{};
    double pending_traj_duration_ = 0.0;
};

}  // namespace franka_rt
