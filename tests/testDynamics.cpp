#include <gtest/gtest.h>
#include "dynamics/SystemFacade.hpp"
#include "dynamics/CR3BP.hpp"

TEST(DynamicsTest, TestGetter) {
    auto system = Dynamics::SystemFacade::createScenario(Dynamics::Scenario::EarthMoonCR3BP_L4);

    ASSERT_EQ(system.getBodies().size(), 3);
    ASSERT_EQ(system.getTime(), 0.);

    EXPECT_NEAR(system.getBodies()[0].getPosition().x(), 0.5 - 0.01215, 1e-5);
}