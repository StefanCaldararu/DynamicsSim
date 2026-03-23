#include <gtest/gtest.h>
#include "dynamics/CR3BP.hpp"
#include <vector>


TEST(CR3BPTest, posDerivativeEqualVel) {
    Dynamics::CR3BPModel model(1.f);

    Eigen::Vector3f pos(1,0,0);
    Eigen::Vector3f vel(1,2,3);
    std::vector<Eigen::Vector3f> positions = {pos};
    std::vector<Eigen::Vector3f> velocities = {vel};

    std::vector<Eigen::Vector3f> dpos_dt;
    std::vector<Eigen::Vector3f> dvel_dt;

    model.derivatives(positions, velocities, dpos_dt, dvel_dt);


    for(int i = 0; i < 3; i++){
        EXPECT_FLOAT_EQ(dpos_dt[0][i], velocities[0][i]);
    }
}

TEST(CR3BPTest, LagrangePoint4Test) {
    Dynamics::CR3BPModel model(0.01215f);

    Eigen::Vector3f pos1(-0.01215f, 0.0f, 0.0f);
    Eigen::Vector3f vel1(0,0,0);
    Eigen::Vector3f pos2(1.0f - 0.01215f, 0.0f, 0.0f);
    Eigen::Vector3f vel2(0,0,0);

    Eigen::Vector3f L4Pos(0.5f - 0.01215f, 0.8660254f, 0.0f);
    Eigen::Vector3f L4Vel(0,0,0);


    std::vector<Eigen::Vector3f> positions = {L4Pos, pos1, pos2};
    std::vector<Eigen::Vector3f> velocities = {L4Vel, vel1, vel2};

    std::vector<Eigen::Vector3f> dpos_dt;
    std::vector<Eigen::Vector3f> dvel_dt;

    model.derivatives(positions, velocities, dpos_dt, dvel_dt);
    
    EXPECT_NEAR(dpos_dt[0][0], 0.f, 1e-5);
    EXPECT_NEAR(dpos_dt[0][1], 0.f, 1e-5);
    EXPECT_NEAR(dpos_dt[0][2], 0.f, 1e-5);
    EXPECT_NEAR(dvel_dt[0][0], 0.f, 1e-5);
    EXPECT_NEAR(dvel_dt[0][1], 0.f, 1e-5);
    EXPECT_NEAR(dvel_dt[0][2], 0.f, 1e-5);
}
