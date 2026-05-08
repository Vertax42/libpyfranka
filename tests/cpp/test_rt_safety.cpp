#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>

#include "realtime_control/rt_common.hpp"

using namespace franka_rt;

namespace {

// Helper: build a 4×4 column-major homogeneous transform from translation
// + axis-angle rotation.  Used to construct test poses without dragging in
// Eigen for what is otherwise a header-only test.
std::array<double, 16> TransformFromAxisAngle(double tx, double ty, double tz,
                                              double ax, double ay, double az,
                                              double angle) {
    double n = std::sqrt(ax*ax + ay*ay + az*az);
    if (n < 1e-12) { ax = 1.0; ay = 0.0; az = 0.0; n = 1.0; }
    ax /= n; ay /= n; az /= n;

    double c = std::cos(angle);
    double s = std::sin(angle);
    double C = 1.0 - c;

    // Column-major 4×4
    std::array<double, 16> T{};
    T[0] = c + ax*ax*C;       T[1] = ay*ax*C + az*s;    T[2] = az*ax*C - ay*s;   T[3] = 0;
    T[4] = ax*ay*C - az*s;    T[5] = c + ay*ay*C;       T[6] = az*ay*C + ax*s;   T[7] = 0;
    T[8] = ax*az*C + ay*s;    T[9] = ay*az*C - ax*s;    T[10] = c + az*az*C;     T[11] = 0;
    T[12] = tx;               T[13] = ty;               T[14] = tz;              T[15] = 1;
    return T;
}

std::array<double, 16> Identity() {
    return TransformFromAxisAngle(0, 0, 0, 0, 0, 1, 0);
}

}  // namespace

// ============================================================================
// CheckFinite / jump detectors
// ============================================================================

TEST(RtSafety, CheckFiniteDetectsNaN) {
    std::array<double, 7> ok{0, 0, 0, 0, 0, 0, 0};
    EXPECT_TRUE(CheckFinite(ok));
    ok[3] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(CheckFinite(ok));
}

TEST(RtSafety, CheckFiniteDetectsInf) {
    std::array<double, 7> bad{0, 0, 0, std::numeric_limits<double>::infinity(),
                              0, 0, 0};
    EXPECT_FALSE(CheckFinite(bad));
}

TEST(RtSafety, CheckFiniteOnPose16) {
    auto T = Identity();
    EXPECT_TRUE(CheckFinite(T));
    T[5] = std::nan("1");
    EXPECT_FALSE(CheckFinite(T));
}

TEST(RtSafety, CheckJointJump) {
    std::array<double, 7> a{}, b{};
    EXPECT_FALSE(CheckJointJump(a, b, 0.01));
    a[2] = 0.05;
    EXPECT_TRUE(CheckJointJump(a, b, 0.01));
    EXPECT_FALSE(CheckJointJump(a, b, 0.1));
}

TEST(RtSafety, CheckCartesianJumpPositionThreshold) {
    auto T0 = Identity();
    auto T1 = TransformFromAxisAngle(0.05, 0, 0, 0, 0, 1, 0);
    EXPECT_TRUE (CheckCartesianJump(T1, T0, 0.01, 1.0));   // pos jump exceeds 1cm
    EXPECT_FALSE(CheckCartesianJump(T1, T0, 0.10, 1.0));   // 10cm threshold OK
}

TEST(RtSafety, CheckCartesianJumpRotationThreshold) {
    auto T0 = Identity();
    auto T1 = TransformFromAxisAngle(0, 0, 0, 0, 0, 1, 0.5);  // 0.5 rad
    EXPECT_TRUE (CheckCartesianJump(T1, T0, 1.0, 0.1));   // rot jump exceeds 0.1 rad
    EXPECT_FALSE(CheckCartesianJump(T1, T0, 1.0, 1.0));   // 1.0 rad threshold OK
}

TEST(RtSafety, CheckCartesianJumpAdaptive30Hz) {
    // Scaled threshold = velocity × interval.
    // At 30 Hz (33 ms): pos thresh = 10 m/s × 0.033 s = 0.33 m.
    auto T0 = Identity();
    auto T_small = TransformFromAxisAngle(0.10, 0, 0, 0, 0, 1, 0);  // 10 cm
    auto T_big   = TransformFromAxisAngle(0.50, 0, 0, 0, 0, 1, 0);  // 50 cm
    double dt = 0.033;
    double pos_thresh = kMaxPositionVelocity * dt;
    double rot_thresh = kMaxRotationVelocity * dt;
    EXPECT_FALSE(CheckCartesianJump(T_small, T0, pos_thresh, rot_thresh));
    EXPECT_TRUE (CheckCartesianJump(T_big,   T0, pos_thresh, rot_thresh));
}

TEST(RtSafety, CheckCartesianJumpAdaptive100Hz) {
    // At 100 Hz (10 ms): pos thresh = 10 m/s × 0.01 s = 0.10 m.
    auto T0 = Identity();
    auto T_small = TransformFromAxisAngle(0.05, 0, 0, 0, 0, 1, 0);
    auto T_big   = TransformFromAxisAngle(0.20, 0, 0, 0, 0, 1, 0);
    double dt = 0.010;
    double pos_thresh = kMaxPositionVelocity * dt;
    double rot_thresh = kMaxRotationVelocity * dt;
    EXPECT_FALSE(CheckCartesianJump(T_small, T0, pos_thresh, rot_thresh));
    EXPECT_TRUE (CheckCartesianJump(T_big,   T0, pos_thresh, rot_thresh));
}

// ============================================================================
// QuatAngularDist
// ============================================================================

TEST(RtSafety, QuatAngularDistIdentity) {
    auto T0 = Identity();
    auto T1 = Identity();
    EXPECT_NEAR(QuatAngularDist(T0, T1), 0.0, 1e-9);
}

TEST(RtSafety, QuatAngularDistAroundZ) {
    auto T0 = Identity();
    for (double a : {0.1, 0.5, 1.0, 2.0, 3.0}) {
        auto T1 = TransformFromAxisAngle(0, 0, 0, 0, 0, 1, a);
        EXPECT_NEAR(QuatAngularDist(T0, T1), a, 1e-6) << "angle=" << a;
    }
}

TEST(RtSafety, QuatAngularDistShortArc) {
    // 350° rotation should be measured as 10° (short arc).
    auto T0 = Identity();
    auto T1 = TransformFromAxisAngle(0, 0, 0, 0, 0, 1,
                                     350.0 * M_PI / 180.0);
    double d = QuatAngularDist(T0, T1);
    EXPECT_NEAR(d, 10.0 * M_PI / 180.0, 1e-3);
}

// ============================================================================
// ClampJointPosition / ClampJointVelocity / ClampJointTorque
// ============================================================================

TEST(RtSafety, ClampJointPositionLimitsHardRange) {
    auto last = std::array<double, 7>{0, 0, 0, -1.0, 0, 1.0, 0};
    auto cmd = last;
    cmd[3] = -100.0;
    cmd[5] = 100.0;
    bool clamped = ClampJointPosition(cmd, last);
    EXPECT_TRUE(clamped);
    EXPECT_GE(cmd[3], kFR3JointMin[3] - 1e-9);
    EXPECT_LE(cmd[5], kFR3JointMax[5] + 1e-9);
}

TEST(RtSafety, ClampJointPositionVelocityCap) {
    // dt=1ms, max_vel default = kFR3JointVelMax[0] = 2.62 rad/s
    // → max delta per cycle = 2.62e-3 rad
    std::array<double, 7> last{0, 0, 0, -1.5, 0, 1.5, 0};
    std::array<double, 7> cmd  = last;
    cmd[0] = 1.0;  // 1 rad jump in one cycle
    bool clamped = ClampJointPosition(cmd, last);
    EXPECT_TRUE(clamped);
    EXPECT_NEAR(cmd[0], last[0] + kFR3JointVelMax[0] * kRtDt, 1e-9);
}

TEST(RtSafety, ClampJointVelocityRespectsAccel) {
    // dt=1ms, max_acc = kFR3JointAccMax[0] = 10 rad/s² → 1e-2 rad/s per cycle
    std::array<double, 7> last_dq{};
    std::array<double, 7> dq{};
    dq[0] = 1.0;  // 1 rad/s jump
    bool clamped = ClampJointVelocity(dq, last_dq);
    EXPECT_TRUE(clamped);
    EXPECT_NEAR(dq[0], 0.0 + kFR3JointAccMax[0] * kRtDt, 1e-9);
}

TEST(RtSafety, ClampJointTorqueRespectsLimit) {
    std::array<double, 7> tau{};
    tau[0] = 1000.0;
    tau[6] = -1000.0;
    bool clamped = ClampJointTorque(tau);
    EXPECT_TRUE(clamped);
    EXPECT_LE(tau[0],  kFR3JointTauMax[0] + 1e-9);
    EXPECT_GE(tau[6], -kFR3JointTauMax[6] - 1e-9);
}

// ============================================================================
// ClampCartesianPose4x4 — translation
// ============================================================================

TEST(RtSafety, ClampCartesianPose4x4SmallStepNoOp) {
    auto last = Identity();
    auto cmd  = TransformFromAxisAngle(1e-4, 0, 0, 0, 0, 1, 0);
    auto cmd0 = cmd;
    bool clamped = ClampCartesianPose4x4(cmd, last);
    EXPECT_FALSE(clamped);
    EXPECT_NEAR(cmd[12], cmd0[12], 1e-12);
}

TEST(RtSafety, ClampCartesianPose4x4ClampsLargeTranslation) {
    auto last = Identity();
    auto cmd  = TransformFromAxisAngle(0.10, 0, 0, 0, 0, 1, 0);  // 10 cm
    bool clamped = ClampCartesianPose4x4(cmd, last,
                                         /*max_lin_vel=*/1.0,
                                         /*max_ang_vel=*/2.0,
                                         /*dt=*/kRtDt);
    EXPECT_TRUE(clamped);
    // |Δp| should now be exactly max_lin_vel × dt = 1e-3 m
    double dx = cmd[12] - last[12];
    double dy = cmd[13] - last[13];
    double dz = cmd[14] - last[14];
    EXPECT_NEAR(std::sqrt(dx*dx + dy*dy + dz*dz), 1.0 * kRtDt, 1e-12);
}

TEST(RtSafety, ClampCartesianPose4x4PreservesTranslationDirection) {
    auto last = Identity();
    auto cmd  = TransformFromAxisAngle(0.05, 0.05, 0, 0, 0, 1, 0);
    ClampCartesianPose4x4(cmd, last, 0.1, 2.0, kRtDt);
    // Direction (1,1,0) should be preserved
    double dx = cmd[12] - last[12];
    double dy = cmd[13] - last[13];
    EXPECT_NEAR(dx, dy, 1e-9);
}

// ============================================================================
// ClampCartesianPose4x4 — rotation (SLERP)
// ============================================================================

TEST(RtSafety, ClampCartesianPose4x4ClampsLargeRotation) {
    auto last = Identity();
    auto cmd  = TransformFromAxisAngle(0, 0, 0, 0, 0, 1, 1.0);  // 1 rad
    bool clamped = ClampCartesianPose4x4(cmd, last,
                                         /*max_lin_vel=*/10.0,
                                         /*max_ang_vel=*/2.0,
                                         /*dt=*/kRtDt);
    EXPECT_TRUE(clamped);
    double measured = QuatAngularDist(cmd, last);
    EXPECT_NEAR(measured, 2.0 * kRtDt, 1e-6);
}

TEST(RtSafety, ClampCartesianPose4x4RotationDotNearOneNoOp) {
    // dot(q1, q2) ≈ 1 — angle effectively zero, no clamp expected
    auto last = Identity();
    auto cmd  = TransformFromAxisAngle(0, 0, 0, 0, 0, 1, 1e-9);
    bool clamped = ClampCartesianPose4x4(cmd, last);
    EXPECT_FALSE(clamped);
}

TEST(RtSafety, ClampCartesianPose4x4RotationShortArc) {
    // Rotate by 350° (which is short-arc -10°). Clamped to max_ang_vel*dt should
    // pick the short arc direction.
    auto last = Identity();
    auto cmd  = TransformFromAxisAngle(0, 0, 0, 0, 0, 1,
                                       350.0 * M_PI / 180.0);
    ClampCartesianPose4x4(cmd, last, 10.0, 2.0, kRtDt);
    double measured = QuatAngularDist(cmd, last);
    EXPECT_NEAR(measured, 2.0 * kRtDt, 1e-6);
}

TEST(RtSafety, ClampCartesianPose4x4BothPosAndRot) {
    auto last = Identity();
    auto cmd  = TransformFromAxisAngle(0.10, 0, 0, 0, 0, 1, 1.0);
    bool clamped = ClampCartesianPose4x4(cmd, last,
                                         /*max_lin_vel=*/1.0,
                                         /*max_ang_vel=*/2.0,
                                         /*dt=*/kRtDt);
    EXPECT_TRUE(clamped);
    double dx = cmd[12] - last[12];
    EXPECT_NEAR(dx, 1e-3, 1e-9);
    EXPECT_NEAR(QuatAngularDist(cmd, last), 2.0 * kRtDt, 1e-6);
}

TEST(RtSafety, ClampCartesianPose4x4Idempotent) {
    auto last = Identity();
    auto cmd1 = TransformFromAxisAngle(0.05, 0, 0, 0, 0, 1, 0.5);
    ClampCartesianPose4x4(cmd1, last, 1.0, 2.0, kRtDt);
    auto cmd2 = cmd1;
    bool again = ClampCartesianPose4x4(cmd2, last, 1.0, 2.0, kRtDt);
    EXPECT_FALSE(again);  // already at limit, not over → no further clamp
    for (int i = 0; i < 16; ++i)
        EXPECT_NEAR(cmd1[i], cmd2[i], 1e-9);
}
