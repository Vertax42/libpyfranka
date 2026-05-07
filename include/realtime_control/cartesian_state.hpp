#pragma once
#include <array>

namespace franka_rt {

/// Snapshot of Cartesian + supporting joint state for Python consumers.
struct CartesianState {
    std::array<double, 16> O_T_EE{};                    // 4×4 column-major
    std::array<double, 6>  O_dP_EE_d{};                 // Desired EE twist (last commanded)
    std::array<double, 6>  K_F_ext_hat_K{};             // Ext wrench in stiffness frame
    std::array<double, 6>  O_F_ext_hat_K{};             // Ext wrench in base frame
    std::array<double, 7>  q{};
    std::array<double, 7>  dq{};
    std::array<double, 7>  tau_ext_hat_filtered{};
    double                 control_command_success_rate = 0.0;
};

}  // namespace franka_rt
