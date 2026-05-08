#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace franka_rt {

// FR3 (Franka Research 3) DoF and per-joint hardware limits.
// Source: Franka FCI documentation (FR3 datasheet, libfranka 0.21).
constexpr int kDoF = 7;

// Joint position limits (q_min, q_max) per joint [rad]
constexpr std::array<double, 7> kFR3JointMin = {
    -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159
};
constexpr std::array<double, 7> kFR3JointMax = {
     2.7437,  1.7837,  2.9007, -0.1518,  2.8065, 4.5169,  3.0159
};
// Joint velocity limits [rad/s] (constant fallback;
// at runtime prefer Robot::getUpperJointVelocityLimits(q) which is
// position-dependent and tighter near the joint limits).
constexpr std::array<double, 7> kFR3JointVelMax = {
    2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26
};
// Joint acceleration limits [rad/s²] — pulled from franka/rate_limiting.h
// kMaxJointAcceleration; conservative use across all joints.
constexpr std::array<double, 7> kFR3JointAccMax = {
    10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0
};
// Joint jerk limits [rad/s³] — franka/rate_limiting.h kMaxJointJerk.
constexpr std::array<double, 7> kFR3JointJerkMax = {
    5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0
};
// Joint torque limits [Nm]
constexpr std::array<double, 7> kFR3JointTauMax = {
    87, 87, 87, 87, 12, 12, 12
};
// Torque rate limits [Nm/s] — franka/rate_limiting.h kMaxTorqueRate.
constexpr std::array<double, 7> kFR3TorqueRateMax = {
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0
};

// Franka FCI Cartesian hard limits — pulled directly from
// franka/rate_limiting.h.  These are the absolute machine ceilings;
// our per-cycle ClampCartesianPose4x4 uses the more conservative kCart*
// constants below.
constexpr double kFR3MaxTransVel  = 3.0;     // m/s
constexpr double kFR3MaxTransAcc  = 9.0;     // m/s²
constexpr double kFR3MaxTransJerk = 4500.0;  // m/s³
constexpr double kFR3MaxRotVel    = 2.5;     // rad/s
constexpr double kFR3MaxRotAcc    = 17.0;    // rad/s²
constexpr double kFR3MaxRotJerk   = 8500.0;  // rad/s³
constexpr double kFR3MaxElbowVel  = 1.5;     // rad/s
constexpr double kFR3MaxElbowAcc  = 10.0;    // rad/s²
constexpr double kFR3MaxElbowJerk = 5000.0;  // rad/s³

// Conservative Cartesian per-cycle clamp limits — well below the FCI
// ceilings to leave headroom for franka::limitRate's three-stage filter.
constexpr double kCartMaxLinearVel  = 1.0;   // m/s
constexpr double kCartMaxAngularVel = 2.0;   // rad/s
constexpr double kCartMaxLinearAcc  = 6.0;   // m/s²
constexpr double kCartMaxAngularAcc = 10.0;  // rad/s²

// 1 kHz cycle time
constexpr double kRtDt = 0.001;

// Adaptive Python→RT jump-detection thresholds (max instantaneous step velocity).
// Multiplied by the measured Python command interval to get per-step thresholds.
//
// Example at 30 Hz (33 ms interval):
//   position:  10 m/s × 0.033 s = 0.33 m  per step
//   rotation:  50 rad/s × 0.033 s = 1.65 rad per step
//   joint:     10 rad/s × 0.033 s = 0.33 rad per step
constexpr double kMaxPositionVelocity = 10.0;   // m/s
constexpr double kMaxRotationVelocity = 50.0;   // rad/s
constexpr double kMaxJointVelocityCmd = 10.0;   // rad/s

constexpr double kMinCommandInterval     = 0.001;
constexpr double kMaxCommandInterval     = 0.1;
constexpr double kDefaultCommandInterval = 0.01;
constexpr auto   kCommandTimeout         = std::chrono::milliseconds(500);

// ---------- Internal math helpers (RT-safe, no Eigen) ----------
namespace detail {

// Decompose a 4×4 column-major homogeneous transform into translation
// vector and unit quaternion [qw, qx, qy, qz].
inline void Mat4ToPosQuat(const std::array<double, 16>& m,
                          std::array<double, 3>& pos,
                          std::array<double, 4>& quat) noexcept {
    pos[0] = m[12]; pos[1] = m[13]; pos[2] = m[14];
    double r00 = m[0], r01 = m[4], r02 = m[8];
    double r10 = m[1], r11 = m[5], r12 = m[9];
    double r20 = m[2], r21 = m[6], r22 = m[10];

    double trace = r00 + r11 + r22;
    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0;
        quat[0] = 0.25 * s;
        quat[1] = (r21 - r12) / s;
        quat[2] = (r02 - r20) / s;
        quat[3] = (r10 - r01) / s;
    } else if ((r00 > r11) && (r00 > r22)) {
        double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
        quat[0] = (r21 - r12) / s;
        quat[1] = 0.25 * s;
        quat[2] = (r01 + r10) / s;
        quat[3] = (r02 + r20) / s;
    } else if (r11 > r22) {
        double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
        quat[0] = (r02 - r20) / s;
        quat[1] = (r01 + r10) / s;
        quat[2] = 0.25 * s;
        quat[3] = (r12 + r21) / s;
    } else {
        double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
        quat[0] = (r10 - r01) / s;
        quat[1] = (r02 + r20) / s;
        quat[2] = (r12 + r21) / s;
        quat[3] = 0.25 * s;
    }

    double n = std::sqrt(quat[0]*quat[0] + quat[1]*quat[1]
                       + quat[2]*quat[2] + quat[3]*quat[3]);
    if (n > 1e-12) {
        double inv = 1.0 / n;
        quat[0] *= inv; quat[1] *= inv; quat[2] *= inv; quat[3] *= inv;
    } else {
        quat = {1.0, 0.0, 0.0, 0.0};
    }
}

// Reassemble a 4×4 column-major homogeneous transform from translation
// and unit quaternion [qw, qx, qy, qz].
inline void PosQuatToMat4(const std::array<double, 3>& pos,
                          const std::array<double, 4>& q,
                          std::array<double, 16>& m) noexcept {
    double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    double xx = qx*qx, yy = qy*qy, zz = qz*qz;
    double xy = qx*qy, xz = qx*qz, yz = qy*qz;
    double wx = qw*qx, wy = qw*qy, wz = qw*qz;

    m[0]  = 1.0 - 2.0*(yy + zz);
    m[1]  = 2.0*(xy + wz);
    m[2]  = 2.0*(xz - wy);
    m[3]  = 0.0;

    m[4]  = 2.0*(xy - wz);
    m[5]  = 1.0 - 2.0*(xx + zz);
    m[6]  = 2.0*(yz + wx);
    m[7]  = 0.0;

    m[8]  = 2.0*(xz + wy);
    m[9]  = 2.0*(yz - wx);
    m[10] = 1.0 - 2.0*(xx + yy);
    m[11] = 0.0;

    m[12] = pos[0]; m[13] = pos[1]; m[14] = pos[2]; m[15] = 1.0;
}

inline double QuatDot(const std::array<double, 4>& a,
                      const std::array<double, 4>& b) noexcept {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

}  // namespace detail

// ---------- Safety helper functions ----------

template <typename Container>
inline bool CheckFinite(const Container& values) noexcept {
    for (const auto& v : values)
        if (!std::isfinite(v)) return false;
    return true;
}

inline bool CheckJointJump(const std::array<double, 7>& cmd,
                           const std::array<double, 7>& last,
                           double threshold) noexcept {
    for (int i = 0; i < 7; ++i)
        if (std::abs(cmd[i] - last[i]) > threshold) return true;
    return false;
}

// 4×4 column-major translation extraction.
inline std::array<double, 3> PoseTranslation(const std::array<double, 16>& T) noexcept {
    return {T[12], T[13], T[14]};
}

// Position distance between two 4×4 column-major poses.
inline double PoseTranslationDist(const std::array<double, 16>& a,
                                  const std::array<double, 16>& b) noexcept {
    double dx = a[12] - b[12], dy = a[13] - b[13], dz = a[14] - b[14];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Quaternion angular distance [rad] between rotations of two 4×4 poses.
inline double QuatAngularDist(const std::array<double, 16>& a,
                              const std::array<double, 16>& b) noexcept {
    std::array<double, 3> pa, pb;
    std::array<double, 4> qa, qb;
    detail::Mat4ToPosQuat(a, pa, qa);
    detail::Mat4ToPosQuat(b, pb, qb);
    double dot = std::abs(detail::QuatDot(qa, qb));
    if (dot > 1.0) dot = 1.0;
    return 2.0 * std::acos(dot);
}

// Returns true if the Cartesian command exceeds the position or rotation
// jump thresholds relative to the previous pose.
inline bool CheckCartesianJump(const std::array<double, 16>& cmd,
                               const std::array<double, 16>& last,
                               double pos_thresh, double rot_thresh) noexcept {
    if (PoseTranslationDist(cmd, last) > pos_thresh) return true;
    if (QuatAngularDist(cmd, last) > rot_thresh) return true;
    return false;
}

// ---------- Per-cycle clamping ----------

inline bool ClampJointPosition(std::array<double, 7>& q,
                               const std::array<double, 7>& last_q,
                               double max_vel = 0,
                               double dt = kRtDt) noexcept {
    bool clamped = false;
    for (int i = 0; i < 7; ++i) {
        double mv = (max_vel > 0) ? max_vel : kFR3JointVelMax[i];
        double max_delta = mv * dt;
        double delta = q[i] - last_q[i];
        if (std::abs(delta) > max_delta) {
            q[i] = last_q[i] + std::copysign(max_delta, delta);
            clamped = true;
        }
        // Hard joint limits
        if (q[i] < kFR3JointMin[i]) { q[i] = kFR3JointMin[i]; clamped = true; }
        if (q[i] > kFR3JointMax[i]) { q[i] = kFR3JointMax[i]; clamped = true; }
    }
    return clamped;
}

// Per-axis joint velocity acceleration clamp (mirrors libpyflexiv).
inline bool ClampJointVelocity(std::array<double, 7>& dq,
                               const std::array<double, 7>& last_dq,
                               double max_acc = 0,
                               double dt = kRtDt) noexcept {
    bool clamped = false;
    for (int i = 0; i < 7; ++i) {
        double a = (max_acc > 0) ? max_acc : kFR3JointAccMax[i];
        double max_dv = a * dt;
        double dv = dq[i] - last_dq[i];
        if (std::abs(dv) > max_dv) {
            dq[i] = last_dq[i] + std::copysign(max_dv, dv);
            clamped = true;
        }
    }
    return clamped;
}

inline bool ClampJointTorque(std::array<double, 7>& tau) noexcept {
    bool clamped = false;
    for (int i = 0; i < 7; ++i) {
        if (tau[i] >  kFR3JointTauMax[i]) { tau[i] =  kFR3JointTauMax[i]; clamped = true; }
        if (tau[i] < -kFR3JointTauMax[i]) { tau[i] = -kFR3JointTauMax[i]; clamped = true; }
    }
    return clamped;
}

// Clamp 4×4 pose change relative to last_T, so the per-cycle translational
// and rotational velocity stay below the given limits.  Translation uses
// Euclidean-distance scaling; rotation uses quaternion SLERP.
inline bool ClampCartesianPose4x4(std::array<double, 16>& T,
                                  const std::array<double, 16>& last_T,
                                  double max_linear_vel  = kCartMaxLinearVel,
                                  double max_angular_vel = kCartMaxAngularVel,
                                  double dt = kRtDt) noexcept {
    bool clamped = false;

    // --- Translation ---
    double max_pos_delta = max_linear_vel * dt;
    double dx = T[12] - last_T[12];
    double dy = T[13] - last_T[13];
    double dz = T[14] - last_T[14];
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist > max_pos_delta && dist > 1e-12) {
        double scale = max_pos_delta / dist;
        T[12] = last_T[12] + dx * scale;
        T[13] = last_T[13] + dy * scale;
        T[14] = last_T[14] + dz * scale;
        clamped = true;
    }

    // --- Rotation: extract quats, SLERP if angular delta too large ---
    std::array<double, 3> p_cur, p_last;
    std::array<double, 4> q_cur, q_last;
    detail::Mat4ToPosQuat(T,      p_cur,  q_cur);
    detail::Mat4ToPosQuat(last_T, p_last, q_last);

    double dot = detail::QuatDot(q_last, q_cur);
    double sign = (dot < 0.0) ? -1.0 : 1.0;
    double adot = std::abs(dot);
    if (adot > 1.0) adot = 1.0;
    double angle = 2.0 * std::acos(adot);

    double max_rot_delta = max_angular_vel * dt;
    if (angle > max_rot_delta && angle > 1e-10) {
        double t = max_rot_delta / angle;
        double half_angle = std::acos(adot);
        double sin_half = std::sin(half_angle);
        std::array<double, 4> q_clamped;
        if (sin_half > 1e-12) {
            double w0 = std::sin((1.0 - t) * half_angle) / sin_half;
            double w1 = std::sin(t * half_angle) / sin_half * sign;
            for (int i = 0; i < 4; ++i)
                q_clamped[i] = w0 * q_last[i] + w1 * q_cur[i];
        } else {
            // Nearly identical — fall back to last orientation
            q_clamped = q_last;
        }
        // Renormalize defensively
        double n = std::sqrt(q_clamped[0]*q_clamped[0] + q_clamped[1]*q_clamped[1]
                           + q_clamped[2]*q_clamped[2] + q_clamped[3]*q_clamped[3]);
        if (n > 1e-12) {
            double inv = 1.0 / n;
            for (int i = 0; i < 4; ++i) q_clamped[i] *= inv;
        }
        // Reassemble 4×4 with clamped rotation but already-clamped translation
        std::array<double, 3> p_after = {T[12], T[13], T[14]};
        detail::PosQuatToMat4(p_after, q_clamped, T);
        clamped = true;
    }

    return clamped;
}

}  // namespace franka_rt
