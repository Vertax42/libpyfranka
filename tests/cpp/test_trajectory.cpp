#include <gtest/gtest.h>
#include <array>
#include <cmath>

#include "realtime_control/trajectory.hpp"

using franka_rt::MinJerkTrajectory;

static std::array<double, 16> IdentityWithTrans(double x, double y, double z) {
    return {1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            x, y, z, 1};
}

// ============================================================================
// MinJerkTrajectory (4×4 pose, min-jerk position basis + SLERP rotation).
// This is the only streaming-trajectory class still used by the SDK; it
// powers CartesianControl::moveToPose.  Streaming itself goes through a
// stateful OTG (PD + jerk-limit), not a re-initable trajectory class.
// ============================================================================

TEST(MinJerkTraj, MonotonicPositionProgress) {
    MinJerkTrajectory t;
    auto T0 = IdentityWithTrans(0, 0, 0);
    auto T1 = IdentityWithTrans(0.2, 0, 0);
    t.init(T0, T1, 1.0);
    std::array<double, 16> T;
    double last_x = 0;
    while (t.step(T)) {
        EXPECT_GE(T[12], last_x - 1e-12);
        last_x = T[12];
    }
    EXPECT_NEAR(T[12], 0.2, 1e-9);
}

TEST(MinJerkTraj, EndpointPrecision) {
    MinJerkTrajectory t;
    auto T0 = IdentityWithTrans(-0.1, 0.05, 0.2);
    auto T1 = IdentityWithTrans(0.3, -0.1, 0.5);
    t.init(T0, T1, 2.0);
    std::array<double, 16> T;
    int steps = 0;
    while (t.step(T) && steps < 5000) steps++;
    EXPECT_NEAR(T[12],  0.3, 1e-9);
    EXPECT_NEAR(T[13], -0.1, 1e-9);
    EXPECT_NEAR(T[14],  0.5, 1e-9);
}

TEST(MinJerkTraj, AutoDurationSensible) {
    auto T0 = IdentityWithTrans(0, 0, 0);
    auto T1 = IdentityWithTrans(0.5, 0, 0);
    double d = MinJerkTrajectory::computeDuration(T0, T1);
    EXPECT_GT(d, 0.5);
    EXPECT_LT(d, 5.0);
}

TEST(MinJerkTraj, CancelStopsImmediately) {
    MinJerkTrajectory t;
    auto T0 = IdentityWithTrans(0, 0, 0);
    auto T1 = IdentityWithTrans(0.2, 0, 0);
    t.init(T0, T1, 1.0);
    std::array<double, 16> T;
    for (int i = 0; i < 10; ++i) ASSERT_TRUE(t.step(T));
    t.cancel();
    EXPECT_FALSE(t.isActive());
    EXPECT_FALSE(t.step(T));
}
