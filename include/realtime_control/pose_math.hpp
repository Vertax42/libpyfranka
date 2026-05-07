#pragma once
#include <array>

namespace franka_rt {

/// Compute 6D pose error between a desired and current 4×4 column-major pose.
/// Returns [Δx, Δy, Δz, ωx, ωy, ωz] where the angular part is the axis-angle
/// representation (axis × angle) of R_err = R_des * R_curr^T.
/// This is the standard task-space error used in Cartesian impedance control.
std::array<double, 6> PoseError(const std::array<double, 16>& desired,
                                const std::array<double, 16>& current) noexcept;

/// Compute end-effector twist (linear + angular velocity in base frame) from
/// joint velocities and the zero Jacobian.
///   J: 6×7 column-major (libfranka's `Model::zeroJacobian` format)
///   dq: 7D joint velocity
/// Returns [vx, vy, vz, ωx, ωy, ωz]
std::array<double, 6> JacobianTwist(const std::array<double, 42>& J,
                                    const std::array<double, 7>& dq) noexcept;

/// Apply the transpose of a 6×7 column-major Jacobian to a 6D wrench.
/// Returns 7D joint torques: tau = J^T * wrench.
std::array<double, 7> JacobianTransposeWrench(const std::array<double, 42>& J,
                                              const std::array<double, 6>& wrench) noexcept;

/// Validate a 4×4 transform: finite, last row [0 0 0 1], rotation orthogonality.
bool IsValidTransform(const std::array<double, 16>& T) noexcept;

}  // namespace franka_rt
