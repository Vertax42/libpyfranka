#include "realtime_control/pose_math.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace franka_rt {

std::array<double, 6> PoseError(const std::array<double, 16>& desired,
                                const std::array<double, 16>& current) noexcept {
    Eigen::Map<const Eigen::Matrix4d> T_des(desired.data());
    Eigen::Map<const Eigen::Matrix4d> T_cur(current.data());

    Eigen::Vector3d pos_err = T_des.block<3, 1>(0, 3) - T_cur.block<3, 1>(0, 3);
    Eigen::Matrix3d R_err = T_des.block<3, 3>(0, 0) * T_cur.block<3, 3>(0, 0).transpose();
    Eigen::AngleAxisd aa(R_err);
    Eigen::Vector3d rot_err = aa.angle() * aa.axis();

    return {pos_err(0), pos_err(1), pos_err(2),
            rot_err(0), rot_err(1), rot_err(2)};
}

std::array<double, 6> JacobianTwist(const std::array<double, 42>& J,
                                    const std::array<double, 7>& dq) noexcept {
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> J_mat(J.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_vec(dq.data());
    Eigen::Matrix<double, 6, 1> twist = J_mat * dq_vec;
    std::array<double, 6> out;
    for (int i = 0; i < 6; ++i) out[i] = twist(i);
    return out;
}

std::array<double, 7> JacobianTransposeWrench(const std::array<double, 42>& J,
                                              const std::array<double, 6>& wrench) noexcept {
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> J_mat(J.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> w_vec(wrench.data());
    Eigen::Matrix<double, 7, 1> tau = J_mat.transpose() * w_vec;
    std::array<double, 7> out;
    for (int i = 0; i < 7; ++i) out[i] = tau(i);
    return out;
}

bool IsValidTransform(const std::array<double, 16>& T) noexcept {
    for (double v : T) if (!std::isfinite(v)) return false;
    // Last row of an SE(3) homogeneous transform must be [0,0,0,1]
    // (column-major: indices 3, 7, 11, 15)
    if (std::abs(T[3])  > 1e-6) return false;
    if (std::abs(T[7])  > 1e-6) return false;
    if (std::abs(T[11]) > 1e-6) return false;
    if (std::abs(T[15] - 1.0) > 1e-6) return false;
    // Rotation block orthogonality (R^T R ≈ I)
    Eigen::Map<const Eigen::Matrix4d> M(T.data());
    Eigen::Matrix3d R = M.block<3, 3>(0, 0);
    Eigen::Matrix3d I3 = R.transpose() * R;
    if ((I3 - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-3) return false;
    return true;
}

}  // namespace franka_rt
