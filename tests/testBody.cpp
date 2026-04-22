#include <gtest/gtest.h>
#include "dynamics/Body.hpp"

using namespace Dynamics;

TEST(BodyTest, InitializationAndGetters) {
    Eigen::Vector3d pos(1,2,3);
    Eigen::Vector3d vel(0,1,0);

    Body b(pos, vel, 5.0, 1.0);

    EXPECT_EQ(b.getPosition(), pos);
    EXPECT_EQ(b.getVelocity(), vel);
    EXPECT_DOUBLE_EQ(b.getMass(), 5.0);
    EXPECT_DOUBLE_EQ(b.getRadius(), 1.0);
}

TEST(BodyTest, StateVectorConstructor) {
    std::vector<double> state = {1,2,3,4,5,6};

    Body b(state, 10.0, 2.0);

    EXPECT_EQ(b.getPosition(), Eigen::Vector3d(1,2,3));
    EXPECT_EQ(b.getVelocity(), Eigen::Vector3d(4,5,6));
}

TEST(BodyTest, SetPositionAndVelocity) {
    Body b(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), 1.0, 1.0);

    b.setPosition(Eigen::Vector3d(10,20,30));
    b.setVelocity(Eigen::Vector3d(1,2,3));

    EXPECT_EQ(b.getPosition(), Eigen::Vector3d(10,20,30));
    EXPECT_EQ(b.getVelocity(), Eigen::Vector3d(1,2,3));
}

TEST(BodyTest, SetAndGetStateVector) {
    Body b(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), 1.0, 1.0);

    std::vector<double> newState = {7,8,9,10,11,12};
    b.setState(newState);

    auto state = b.getState();

    EXPECT_EQ(state, newState);
}

TEST(BodyTest, InvalidStateThrows) {
    std::vector<double> badState = {1,2,3};

    EXPECT_THROW(
        Body b(badState, 1.0, 1.0),
        std::invalid_argument
    );
}