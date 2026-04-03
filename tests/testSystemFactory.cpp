#include <gtest/gtest.h>
#include "dynamics/SystemFactory.hpp"
#include "dynamics/CR3BP.hpp"

TEST(SystemFactoryTest, EarthMoonL4Scenario) {
    auto system = Dynamics::SystemFactory::createEarthMoonCR3BP_L4();

    ASSERT_EQ(system.getBodies().size(), 3);
    EXPECT_NEAR(system.getBodies()[0].getPosition().x(), 0.5 - 0.01215, 1e-5);
}

TEST(SystemFactoryTest, TwoBodyMutualScenario) {
    auto system = Dynamics::SystemFactory::createTwoBodyMutual();

    ASSERT_EQ(system.getBodies().size(), 2);
    EXPECT_NEAR(system.getBodies()[0].getMass(), 10.0, 1e-5);
}

TEST(SystemFactoryTest, OneLargeOneSmallScenario) {
    auto system = Dynamics::SystemFactory::createOneLargeOneSmall();

    ASSERT_EQ(system.getBodies().size(), 2);
    EXPECT_GT(system.getBodies()[0].getMass(), system.getBodies()[1].getMass());
}

TEST(SystemFactoryTest, ThreeBodyStableScenario) {
    auto system = Dynamics::SystemFactory::createThreeBodyStable();

    ASSERT_EQ(system.getBodies().size(), 3);
    EXPECT_EQ(system.getBodies()[0].getMass(), 1.0);
}

TEST(SystemFactoryTest, CR3BPLEOScenario) {
    auto system = Dynamics::SystemFactory::createCR3BPLEO();

    ASSERT_EQ(system.getBodies().size(), 3);
    EXPECT_NEAR(system.getBodies()[0].getPosition().y(), 0.0, 1e-4f);
}
