#include <gtest/gtest.h>
#include "dynamics/Body.hpp"


TEST(BodyTest, Initialization) {
    Eigen::Vector3f pos(1,2,3);
    Eigen::Vector3f vel(0,1,0);

    Dynamics::Body b(pos, vel, 5.0f, 1.0f);

    EXPECT_EQ(b.getPosition(), pos);
    EXPECT_EQ(b.getVelocity(), vel);
    EXPECT_FLOAT_EQ(b.getMass(), 5.0f);
}