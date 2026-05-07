#include <gtest/gtest.h>
#include <array>
#include <cmath>

#include "realtime_control/trajectory.hpp"

using franka_rt::JointLinearTrajectory;
using franka_rt::LinearTrajectory;
using franka_rt::MinJerkTrajectory;

static std::array<double, 16> IdentityWithTrans(double x, double y, double z) {
    return {1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            x, y, z, 1};
}

TEST(JointLinearTraj, ReachesTargetAfterDuration) {
    JointLinearTrajectory t;
    std::array<double, 7> q0{0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> q1{0.5, 0, 0, 0, 0, 0, 0};
    t.init(q0, q1, 1.0);  // 1s
    std::array<double, 7> q;
    bool active = true;
    int steps = 0;
    while (active) {
        active = t.step(q);
        steps++;
        if (steps > 2000) break;
    }
    for (int i = 0; i < 7; ++i)
        EXPECT_NEAR(q[i], q1[i], 1e-9);
    EXPECT_LE(steps, 1010);
    EXPECT_GE(steps, 990);
}

TEST(JointLinearTraj, CancelStopsImmediately) {
    JointLinearTrajectory t;
    std::array<double, 7> q0{}, q1{1, 0, 0, 0, 0, 0, 0};
    t.init(q0, q1, 1.0);
    std::array<double, 7> q;
    for (int i = 0; i < 100; ++i) t.step(q);
    t.cancel();
    EXPECT_FALSE(t.step(q));
    EXPECT_FALSE(t.isActive());
}

TEST(LinearTraj, PoseReachedAfterDuration) {
    LinearTrajectory t;
    auto T0 = IdentityWithTrans(0, 0, 0);
    auto T1 = IdentityWithTrans(0.1, 0, 0);
    t.init(T0, T1, 0.5);
    std::array<double, 16> T;
    bool active = true;
    int steps = 0;
    while (active && steps < 1000) {
        active = t.step(T);
        steps++;
    }
    EXPECT_NEAR(T[12], 0.1, 1e-9);
    EXPECT_NEAR(T[13], 0.0, 1e-9);
}

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
