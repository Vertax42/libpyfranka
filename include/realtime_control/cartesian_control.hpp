#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <franka/robot.h>

#include "realtime_control/cartesian_state.hpp"
#include "realtime_control/shared_memory.hpp"
#include "realtime_control/trajectory.hpp"

namespace franka_rt {

/// Selects which libfranka built-in controller tracks the streamed pose.
/// Naming mirrors pylibfranka.ControllerMode (no k prefix; member names
/// match upstream).
enum class ControllerMode {
    /// libfranka kJointImpedance: rigid joint-space tracking after IK.
    /// Better for precise pose-to-pose motion / trajectory replay.
    /// Stiffness via Robot::setJointImpedance(K_theta), 7D.
    JointImpedance,
    /// libfranka kCartesianImpedance: end-effector compliance in Cartesian
    /// directions, suited for teleop and contact tasks.  Stiffness via
    /// Robot::setCartesianImpedance(K_x), 6D, range [10,3000] N/m for
    /// xyz and [1,300] Nm/rad for rpy.
    CartesianImpedance,
};

/// One Python→RT command queue entry.  Includes a timestamp so the RT
/// thread can derive an adaptive jump threshold without coordinating
/// with the producer.
struct PoseCommand {
    std::array<double, 16> T{};
    std::array<double, 2>  elbow{};
    bool                   has_elbow = false;
    std::chrono::steady_clock::time_point timestamp;
};

/// 1 kHz Cartesian-pose streaming controller.
///
/// Architecture:
///   Python `setTargetPose(...)` → SPSC ring buffer → libfranka
///   robot.control(callback, mode, limit_rate=true) at 1 kHz → continuous
///   jerk-limited OTG follows sparse user commands → libfranka's internal
///   kCartesianImpedance / kJointImpedance controller does the actual IK +
///   tracking → motors.
///
/// Safety mechanisms (all run every 1 ms cycle):
///   - finite check + isHomogeneousTransformation on every input
///   - adaptive jump detection scaled by measured Python command interval
///   - continuous velocity/acceleration state; no segment-boundary re-init
///   - state.current_errors triggers immediate e-stop (hold last pose)
///   - command timeout: hold last pose forever (no e-stop)
class CartesianControl {
public:
    explicit CartesianControl(franka::Robot& robot,
                              ControllerMode mode = ControllerMode::CartesianImpedance);
    ~CartesianControl();

    CartesianControl(const CartesianControl&) = delete;
    CartesianControl& operator=(const CartesianControl&) = delete;

    // ---- Lifecycle ----
    void start();
    void stop();
    bool isRunning() const noexcept { return running_.load(); }
    void triggerEstop() noexcept { estop_.store(true); }

    // ---- Streaming target API ----
    void setTargetPose(const std::array<double, 16>& T) noexcept;
    void setTargetPoseWithElbow(const std::array<double, 16>& T,
                                const std::array<double, 2>& elbow) noexcept;
    CartesianState getState() const noexcept;

    // ---- Reset trajectory: non-blocking RT-mode min-jerk ----
    void moveToPose(const std::array<double, 16>& T,
                    double duration_sec = 0.0) noexcept;
    bool isMoving() const noexcept { return moving_.load(); }
    void cancelMove() noexcept;

    ControllerMode mode() const noexcept { return mode_; }

private:
    void rtLoop();

    franka::Robot& robot_;
    ControllerMode mode_;
    std::thread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> estop_{false};
    std::atomic<bool> moving_{false};
    std::atomic<bool> stopped_{false};  // idempotent stop guard

    // Command pipe: SPSC ring buffer (Python writer → RT reader).
    RealTimeBuffer<PoseCommand, 8> cmd_buf_;

    // moveToPose request (single-pending).  The RT callback only uses
    // try_lock() on this mutex so Python cannot stall the 1 kHz loop.
    mutable std::mutex pending_move_mtx_;
    std::array<double, 16> pending_move_target_{};
    double                 pending_move_duration_ = 0.0;
    bool                   pending_move_init_ = false;

    // Latest snapshot of robot state for Python reads.  The RT callback
    // updates it opportunistically with try_lock() to avoid priority inversion.
    mutable std::mutex state_mtx_;
    CartesianState latest_state_{};
};

}  // namespace franka_rt
