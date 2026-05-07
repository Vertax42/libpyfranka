#pragma once
#include <array>

namespace franka_rt {

/// Snapshot of joint-space + minimal task-space robot state, copied out of a
/// franka::RobotState under the controller mutex for safe Python access.
struct JointState {
    std::array<double, 7>  q{};                         // Measured joint positions [rad]
    std::array<double, 7>  dq{};                        // Measured joint velocities [rad/s]
    std::array<double, 7>  tau_J{};                     // Measured joint torques [Nm]
    std::array<double, 7>  tau_ext_hat_filtered{};      // Estimated external joint torques [Nm]
    std::array<double, 16> O_T_EE{};                    // EE pose (4×4 column-major)
    double                 control_command_success_rate = 0.0;
};

}  // namespace franka_rt
