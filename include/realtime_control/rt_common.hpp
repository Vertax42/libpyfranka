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
// Joint velocity limits [rad/s]
constexpr std::array<double, 7> kFR3JointVelMax = {
    2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26
};
// Joint torque limits [Nm]
constexpr std::array<double, 7> kFR3JointTauMax = {
    87, 87, 87, 87, 12, 12, 12
};

// Cartesian limits (FR3 / FCI defaults — keep conservative for impedance).
constexpr double kCartMaxLinearVel  = 1.0;   // m/s
constexpr double kCartMaxAngularVel = 2.5;   // rad/s
constexpr double kCartMaxLinearAcc  = 6.0;   // m/s²
constexpr double kCartMaxAngularAcc = 10.0;  // rad/s²

// 1 kHz cycle time
constexpr double kRtDt = 0.001;

// Adaptive Python→RT jump-detection thresholds (max instantaneous step velocity).
// Multiplied by the measured Python command interval to get per-step thresholds.
constexpr double kMaxPositionVelocity = 10.0;   // m/s
constexpr double kMaxRotationVelocity = 50.0;   // rad/s
constexpr double kMaxJointVelocityCmd = 10.0;   // rad/s

constexpr double kMinCommandInterval     = 0.001;
constexpr double kMaxCommandInterval     = 0.1;
constexpr double kDefaultCommandInterval = 0.01;
constexpr auto   kCommandTimeout         = std::chrono::milliseconds(500);

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

inline bool ClampJointTorque(std::array<double, 7>& tau) noexcept {
    bool clamped = false;
    for (int i = 0; i < 7; ++i) {
        if (tau[i] >  kFR3JointTauMax[i]) { tau[i] =  kFR3JointTauMax[i]; clamped = true; }
        if (tau[i] < -kFR3JointTauMax[i]) { tau[i] = -kFR3JointTauMax[i]; clamped = true; }
    }
    return clamped;
}

}  // namespace franka_rt
