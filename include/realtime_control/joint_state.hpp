#pragma once
#include <array>
#include <cstdint>

namespace franka_rt {

/// Robot mode mirror of franka::RobotMode for Python consumers.
/// Naming mirrors pylibfranka.RobotMode (no k prefix, member names: Idle,
/// Move, Reflex, etc.) to allow drop-in API compatibility.
enum class RobotMode : uint8_t {
    Other                  = 0,
    Idle                   = 1,
    Move                   = 2,
    Guiding                = 3,
    Reflex                 = 4,
    UserStopped            = 5,
    AutomaticErrorRecovery = 6,
};

/// Snapshot of joint-space + minimal task-space robot state, copied out of a
/// franka::RobotState under the controller mutex for safe Python access.
///
/// Layout mirrors the most useful subset of franka::RobotState (libfranka
/// 0.21).  See robot_state.h for full field semantics.
struct JointState {
    // ---- Measured ----
    std::array<double, 7>  q{};                      // measured joint position [rad]
    std::array<double, 7>  dq{};                     // measured joint velocity [rad/s]
    std::array<double, 7>  tau_J{};                  // measured joint torque [Nm]
    std::array<double, 7>  dtau_J{};                 // d/dt of tau_J [Nm/s]
    std::array<double, 7>  tau_ext_hat_filtered{};   // estimated external torque [Nm]
    std::array<double, 16> O_T_EE{};                 // measured EE pose, 4×4 col-major

    // ---- Last desired (output of libfranka's internal controller) ----
    std::array<double, 7>  q_d{};                    // desired joint position [rad]
    std::array<double, 7>  dq_d{};                   // desired joint velocity [rad/s]
    std::array<double, 7>  ddq_d{};                  // desired joint acceleration [rad/s²]
    std::array<double, 7>  tau_J_d{};                // desired link-side torque [Nm]

    // ---- External wrench at the EE (for monitoring / GUI) ----
    std::array<double, 6>  O_F_ext_hat_K{};          // ext wrench in base frame
    std::array<double, 6>  K_F_ext_hat_K{};          // ext wrench in stiffness frame

    // ---- Status / errors ----
    // bit-packed view of franka::Errors (41 flags).  Bit indices follow
    // the declaration order in franka/errors.h.  Use Python-side helper
    // to decode into named flags.
    uint64_t   current_errors        = 0;
    uint64_t   last_motion_errors    = 0;
    RobotMode  robot_mode            = RobotMode::Other;
    double     control_command_success_rate = 0.0;
};

}  // namespace franka_rt
