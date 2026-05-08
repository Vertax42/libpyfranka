#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace franka_rt {

// RT-safe trajectory generators that operate on 4×4 column-major homogeneous
// transforms (libfranka's native pose format).  Internally we factor the pose
// into a 3D position and a quaternion (qw, qx, qy, qz) for SLERP, then
// reassemble the 4×4 result.  All operations are noexcept and zero-heap.

namespace detail {

inline void mat4ToPosQuat(const std::array<double, 16>& m,
                          std::array<double, 3>& pos,
                          std::array<double, 4>& quat) noexcept {
    // Column-major: m[0..3] = column 0, m[4..7] = column 1, etc.
    // Translation is column 3.
    pos[0] = m[12];
    pos[1] = m[13];
    pos[2] = m[14];

    // Rotation 3×3 (column-major):
    //   r00 = m[0],  r10 = m[1],  r20 = m[2]
    //   r01 = m[4],  r11 = m[5],  r21 = m[6]
    //   r02 = m[8],  r12 = m[9],  r22 = m[10]
    double r00 = m[0], r01 = m[4], r02 = m[8];
    double r10 = m[1], r11 = m[5], r12 = m[9];
    double r20 = m[2], r21 = m[6], r22 = m[10];

    double trace = r00 + r11 + r22;
    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0;  // s = 4*qw
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

    // Normalize
    double n = std::sqrt(quat[0]*quat[0] + quat[1]*quat[1]
                       + quat[2]*quat[2] + quat[3]*quat[3]);
    if (n > 1e-12) {
        double inv = 1.0 / n;
        quat[0] *= inv; quat[1] *= inv; quat[2] *= inv; quat[3] *= inv;
    } else {
        quat = {1.0, 0.0, 0.0, 0.0};
    }
}

inline void posQuatToMat4(const std::array<double, 3>& pos,
                          const std::array<double, 4>& q,
                          std::array<double, 16>& m) noexcept {
    double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    double xx = qx*qx, yy = qy*qy, zz = qz*qz;
    double xy = qx*qy, xz = qx*qz, yz = qy*qz;
    double wx = qw*qx, wy = qw*qy, wz = qw*qz;

    // Column-major
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

    m[12] = pos[0];
    m[13] = pos[1];
    m[14] = pos[2];
    m[15] = 1.0;
}

inline double quatDot(const std::array<double, 4>& a,
                      const std::array<double, 4>& b) noexcept {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

inline void slerp(const std::array<double, 4>& q0,
                  std::array<double, 4> q1,
                  double t,
                  std::array<double, 4>& out) noexcept {
    double dot = quatDot(q0, q1);
    if (dot < 0.0) {
        q1 = {-q1[0], -q1[1], -q1[2], -q1[3]};
        dot = -dot;
    }
    if (dot > 1.0) dot = 1.0;
    double theta = std::acos(dot);
    if (theta < 1e-6) {
        for (int i = 0; i < 4; ++i) out[i] = q0[i] + t * (q1[i] - q0[i]);
        double n = std::sqrt(out[0]*out[0]+out[1]*out[1]+out[2]*out[2]+out[3]*out[3]);
        if (n > 1e-12) { double inv=1.0/n; for (int i=0;i<4;++i) out[i]*=inv; }
        return;
    }
    double sin_t = std::sin(theta);
    double w0 = std::sin((1.0 - t) * theta) / sin_t;
    double w1 = std::sin(t * theta) / sin_t;
    for (int i = 0; i < 4; ++i) out[i] = w0 * q0[i] + w1 * q1[i];
}

}  // namespace detail

/// Minimum-jerk pose trajectory.
/// Position uses min-jerk basis s(τ) = 10τ³ - 15τ⁴ + 6τ⁵.
/// Rotation uses SLERP with parameter s(τ).
class MinJerkTrajectory {
public:
    static constexpr double kMaxLinearVel  = 0.5;   // m/s
    static constexpr double kMaxAngularVel = 1.5;   // rad/s
    static constexpr double kMinDuration   = 1.0;   // s

    static double computeDuration(const std::array<double, 16>& start,
                                  const std::array<double, 16>& end,
                                  double max_lin = kMaxLinearVel,
                                  double max_ang = kMaxAngularVel,
                                  double min_dur = kMinDuration) noexcept {
        std::array<double, 3> p0, p1; std::array<double, 4> q0, q1;
        detail::mat4ToPosQuat(start, p0, q0);
        detail::mat4ToPosQuat(end, p1, q1);
        double d2 = 0;
        for (int i = 0; i < 3; ++i) { double d = p1[i] - p0[i]; d2 += d * d; }
        double pd = std::sqrt(d2);
        double dot = std::abs(detail::quatDot(q0, q1));
        if (dot > 1.0) dot = 1.0;
        double rd = 2.0 * std::acos(dot);
        constexpr double kPeak = 1.875;
        double t_p = (max_lin > 0) ? kPeak * pd / max_lin : 0.0;
        double t_r = (max_ang > 0) ? kPeak * rd / max_ang : 0.0;
        return std::max({t_p, t_r, min_dur});
    }

    void init(const std::array<double, 16>& start_pose,
              const std::array<double, 16>& end_pose,
              double duration_sec) noexcept {
        detail::mat4ToPosQuat(start_pose, p0_, q0_);
        detail::mat4ToPosQuat(end_pose, p1_, q1_);
        // Short-arc
        if (detail::quatDot(q0_, q1_) < 0.0) {
            for (int i = 0; i < 4; ++i) q1_[i] = -q1_[i];
        }
        if (duration_sec <= 0.0) duration_sec = computeDuration(start_pose, end_pose);
        total_steps_ = std::max<uint32_t>(1, static_cast<uint32_t>(duration_sec * 1000.0));
        current_step_ = 0;
        active_ = true;
        cancelled_ = false;
    }

    bool step(std::array<double, 16>& out_pose) noexcept {
        if (!active_ || cancelled_) return false;
        current_step_++;
        if (current_step_ >= total_steps_) {
            detail::posQuatToMat4(p1_, q1_, out_pose);
            active_ = false;
            return false;
        }
        double tau = static_cast<double>(current_step_) / total_steps_;
        double tau2 = tau * tau, tau3 = tau2 * tau;
        double s = 10.0 * tau3 - 15.0 * tau3 * tau + 6.0 * tau3 * tau2;
        std::array<double, 3> p;
        for (int i = 0; i < 3; ++i) p[i] = p0_[i] + s * (p1_[i] - p0_[i]);
        std::array<double, 4> q;
        detail::slerp(q0_, q1_, s, q);
        detail::posQuatToMat4(p, q, out_pose);
        return true;
    }

    void cancel() noexcept { cancelled_ = true; active_ = false; }
    bool isActive() const noexcept { return active_ && !cancelled_; }

private:
    std::array<double, 3> p0_{}, p1_{};
    std::array<double, 4> q0_{1, 0, 0, 0}, q1_{1, 0, 0, 0};
    uint32_t total_steps_ = 0, current_step_ = 0;
    bool active_ = false, cancelled_ = false;
};

}  // namespace franka_rt
