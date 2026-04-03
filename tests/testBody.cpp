#include <gtest/gtest.h>
#include "dynamics/Body.hpp"


TEST(BodyTest, Initialization) {
    Eigen::Vector3d pos(1,2,3);
    Eigen::Vector3d vel(0,1,0);

    Dynamics::Body b(pos, vel, 5.0, 1.0);

    EXPECT_EQ(b.getPosition(), pos);
    EXPECT_EQ(b.getVelocity(), vel);
    EXPECT_DOUBLE_EQ(b.getMass(), 5.0);
}