#include <gtest/gtest.h>
#include "dynamics/SystemFacade.hpp"
#include "dynamics/CR3BP.hpp"

TEST(SystemFacadeTest, EarthMoonL4Scenario) {
    auto system = Dynamics::SystemFacade::createScenario(Dynamics::Scenario::EarthMoonCR3BP_L4);

    ASSERT_EQ(system.getBodies().size(), 3);
    EXPECT_NEAR(system.getBodies()[0].getPosition().x(), 0.5 - 0.01215, 1e-5);
}

TEST(SystemFacadeTest, EarthMoonCR3BP_L4_Tadpole){
    auto system = Dynamics::SystemFacade::createScenario(Dynamics::Scenario::EarthMoonCR3BP_L4_Tadpole);

    ASSERT_EQ(system.getBodies().size(), 3);
}

TEST(SystemFacadeTest, TwoBodyMutualScenario) {
    auto system = Dynamics::SystemFacade::createScenario(Dynamics::Scenario::TwoBodyMutual);

    ASSERT_EQ(system.getBodies().size(), 2);
    EXPECT_NEAR(system.getBodies()[0].getMass(), 10.0, 1e-5);
}

TEST(SystemFacadeTest, OneLargeOneSmallScenario) {
    auto system = Dynamics::SystemFacade::createScenario(Dynamics::Scenario::OneLargeOneSmall);

    ASSERT_EQ(system.getBodies().size(), 2);
    EXPECT_GT(system.getBodies()[0].getMass(), system.getBodies()[1].getMass());
}

TEST(SystemFacadeTest, ThreeBodyStableScenario) {
    auto system = Dynamics::SystemFacade::createScenario(Dynamics::Scenario::ThreeBodyStable);
    ASSERT_EQ(system.getBodies().size(), 3);
    EXPECT_EQ(system.getBodies()[0].getMass(), 1.0);
}
