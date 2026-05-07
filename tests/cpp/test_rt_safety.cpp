#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>

#include "realtime_control/rt_common.hpp"

using namespace franka_rt;

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

TEST(RtSafety, CheckJointJump) {
    std::array<double, 7> a{}, b{};
    EXPECT_FALSE(CheckJointJump(a, b, 0.01));
    a[2] = 0.05;
    EXPECT_TRUE(CheckJointJump(a, b, 0.01));
    EXPECT_FALSE(CheckJointJump(a, b, 0.1));
}

TEST(RtSafety, ClampJointPositionLimitsHardRange) {
    auto last = std::array<double, 7>{0, 0, 0, -1.0, 0, 1.0, 0};
    auto cmd = last;
    cmd[3] = -100.0;  // way past min
    cmd[5] = 100.0;   // way past max
    bool clamped = ClampJointPosition(cmd, last);
    EXPECT_TRUE(clamped);
    EXPECT_GE(cmd[3], kFR3JointMin[3] - 1e-9);
    EXPECT_LE(cmd[5], kFR3JointMax[5] + 1e-9);
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
