#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <franka/robot.h>

#include "realtime_control/joint_state.hpp"
#include "realtime_control/shared_memory.hpp"
#include "realtime_control/trajectory.hpp"

namespace franka_rt {

/// One Python→RT joint command queue entry.
struct JointCommand {
    std::array<double, 7> q{};
    std::chrono::steady_clock::time_point timestamp;
};

/// 1 kHz joint-position streaming controller.
///
/// Architecture (mirrors CartesianControl):
///   Python `setTargetJoints(...)` → SPSC ring buffer → libfranka 1 kHz
///   callback → continuous online trajectory generator drives `stream_q`
///   toward the latest target with bounded jerk → libfranka's internal
///   joint-impedance controller does the actual tracking.
///
/// We use `robot.control(callback, kJointImpedance, limit_rate=true)` rather
/// than `startJointPositionControl + readOnce/writeOnce`.  Reasons documented
/// in CartesianControl::rtLoop().
class JointPositionControl {
public:
    explicit JointPositionControl(franka::Robot& robot);
    ~JointPositionControl();

    JointPositionControl(const JointPositionControl&) = delete;
    JointPositionControl& operator=(const JointPositionControl&) = delete;

    // ---- Lifecycle ----
    void start();
    void stop();
    bool isRunning() const noexcept { return running_.load(); }
    void triggerEstop() noexcept { estop_.store(true); }

    // ---- Streaming target API ----
    void setTargetJoints(const std::array<double, 7>& q) noexcept;
    JointState getState() const noexcept;

    // ---- Reset trajectory: non-blocking RT-mode min-jerk move ----
    void moveToJoints(const std::array<double, 7>& q,
                      double duration_sec = 0.0) noexcept;
    bool isMoving() const noexcept { return moving_.load(); }
    void cancelMove() noexcept;

private:
    void rtLoop();

    franka::Robot& robot_;
    std::thread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> estop_{false};
    std::atomic<bool> moving_{false};
    std::atomic<bool> stopped_{false};

    // SPSC ring buffer (Python writer → RT reader).
    RealTimeBuffer<JointCommand, 8> cmd_buf_;

    // moveToJoints request (single-pending).  The RT callback only uses
    // try_lock() on this mutex so Python cannot stall the 1 kHz loop.
    mutable std::mutex pending_move_mtx_;
    std::array<double, 7> pending_move_target_{};
    double                pending_move_duration_ = 0.0;
    bool                  pending_move_init_ = false;

    // Latest snapshot of robot state for Python reads.  The RT callback
    // updates it opportunistically with try_lock() to avoid priority inversion.
    mutable std::mutex state_mtx_;
    JointState latest_state_{};
};

}  // namespace franka_rt
