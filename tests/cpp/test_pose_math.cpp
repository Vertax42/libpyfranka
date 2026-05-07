#include <gtest/gtest.h>
#include <array>
#include <cmath>

#include "realtime_control/pose_math.hpp"

using franka_rt::IsValidTransform;
using franka_rt::PoseError;

static std::array<double, 16> Identity() {
    return {1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1};
}

TEST(PoseMath, IdentityIsValid) {
    EXPECT_TRUE(IsValidTransform(Identity()));
}

TEST(PoseMath, NonOrthogonalRotationRejected) {
    auto T = Identity();
    T[0] = 2.0;  // R(0,0) = 2 — not orthogonal
    EXPECT_FALSE(IsValidTransform(T));
}

TEST(PoseMath, ErrorOfIdenticalPosesIsZero) {
    // Translation-only pose
    std::array<double, 16> T = Identity();
    T[12] = 0.5; T[13] = -0.3; T[14] = 0.2;
    auto err = PoseError(T, T);
    for (double v : err) EXPECT_NEAR(v, 0.0, 1e-12);
}

TEST(PoseMath, PositionErrorMatchesTranslationDelta) {
    auto a = Identity();
    auto b = Identity();
    a[12] = 1.0; b[12] = 1.5;
    auto err = PoseError(a, b);  // desired=a, current=b -> Δ = a - b
    EXPECT_NEAR(err[0], -0.5, 1e-12);
    EXPECT_NEAR(err[1],  0.0, 1e-12);
    EXPECT_NEAR(err[2],  0.0, 1e-12);
    EXPECT_NEAR(err[3],  0.0, 1e-12);
    EXPECT_NEAR(err[4],  0.0, 1e-12);
    EXPECT_NEAR(err[5],  0.0, 1e-12);
}

TEST(PoseMath, RotationAroundZGivesAxisAngleZ) {
    // 90° rotation about Z (column-major)
    auto cur = Identity();
    auto des = Identity();
    double c = 0.0, s = 1.0;
    des[0] =  c; des[1] = s; des[4] = -s; des[5] = c;
    auto err = PoseError(des, cur);
    // Expected: axis-angle = (0, 0, π/2)
    EXPECT_NEAR(err[3], 0.0, 1e-9);
    EXPECT_NEAR(err[4], 0.0, 1e-9);
    EXPECT_NEAR(err[5], M_PI / 2.0, 1e-9);
}
