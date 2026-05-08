#pragma once
#include <array>
#include <cstdint>

#include "realtime_control/joint_state.hpp"  // for RobotMode

namespace franka_rt {

/// Snapshot of Cartesian + supporting joint state for Python consumers.
///
/// Mirrors franka::RobotState fields most useful for Cartesian-space
/// teleop / VLA replay.
struct CartesianState {
    // ---- Measured ----
    std::array<double, 16> O_T_EE{};                 // measured EE pose, 4×4 col-major
    std::array<double, 7>  q{};                      // joint position [rad]
    std::array<double, 7>  dq{};                     // joint velocity [rad/s]
    std::array<double, 2>  elbow{};                  // measured elbow [J3 rad, J4 sign]

    // ---- Last desired (output of libfranka's internal motion generator) ----
    std::array<double, 16> O_T_EE_d{};               // desired EE pose
    std::array<double, 6>  O_dP_EE_d{};              // desired EE twist
    std::array<double, 7>  q_d{};                    // desired joint position (IK output)
    std::array<double, 7>  dq_d{};                   // desired joint velocity
    std::array<double, 2>  elbow_d{};

    // ---- Last commanded (what we wrote in the previous cycle) ----
    std::array<double, 16> O_T_EE_c{};               // last commanded EE pose
    std::array<double, 6>  O_dP_EE_c{};              // last commanded EE twist
    std::array<double, 6>  O_ddP_EE_c{};             // last commanded EE acceleration
    std::array<double, 2>  elbow_c{};

    // ---- Wrench / forces ----
    std::array<double, 6>  O_F_ext_hat_K{};          // ext wrench in base frame
    std::array<double, 6>  K_F_ext_hat_K{};          // ext wrench in stiffness frame
    std::array<double, 7>  tau_ext_hat_filtered{};

    // ---- Status / errors ----
    uint64_t   current_errors        = 0;
    uint64_t   last_motion_errors    = 0;
    RobotMode  robot_mode            = RobotMode::Other;
    double     control_command_success_rate = 0.0;
};

}  // namespace franka_rt
