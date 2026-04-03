#include <gtest/gtest.h>
#include "dynamics/CR3BP.hpp"
#include <vector>


TEST(CR3BPTest, posDerivativeEqualVel) {
    Dynamics::CR3BPModel model(1.0);

    Eigen::Vector3d pos(1,0,0);
    Eigen::Vector3d vel(1,2,3);
    std::vector<Eigen::Vector3d> positions = {pos};
    std::vector<Eigen::Vector3d> velocities = {vel};

    std::vector<Eigen::Vector3d> dpos_dt;
    std::vector<Eigen::Vector3d> dvel_dt;

    model.derivatives(0.0, positions, velocities, dpos_dt, dvel_dt);


    for(int i = 0; i < 3; i++){
        EXPECT_DOUBLE_EQ(dpos_dt[0][i], velocities[0][i]);
    }
}

TEST(CR3BPTest, LagrangePoint4Test) {
    Dynamics::CR3BPModel model(0.01215);

    Eigen::Vector3d pos1(-0.01215, 0.0, 0.0);
    Eigen::Vector3d vel1(0,0,0);
    Eigen::Vector3d pos2(1.0 - 0.01215, 0.0, 0.0);
    Eigen::Vector3d vel2(0,0,0);

    Eigen::Vector3d L4Pos(0.5 - 0.01215, 0.8660254, 0.0);
    Eigen::Vector3d L4Vel(0,0,0);


    std::vector<Eigen::Vector3d> positions = {L4Pos, pos1, pos2};
    std::vector<Eigen::Vector3d> velocities = {L4Vel, vel1, vel2};

    std::vector<Eigen::Vector3d> dpos_dt;
    std::vector<Eigen::Vector3d> dvel_dt;

    model.derivatives(0.0, positions, velocities, dpos_dt, dvel_dt);
    
    EXPECT_NEAR(dpos_dt[0][0], 0.0, 1e-5);
    EXPECT_NEAR(dpos_dt[0][1], 0.0, 1e-5);
    EXPECT_NEAR(dpos_dt[0][2], 0.0, 1e-5);
    EXPECT_NEAR(dvel_dt[0][0], 0.0, 1e-5);
    EXPECT_NEAR(dvel_dt[0][1], 0.0, 1e-5);
    EXPECT_NEAR(dvel_dt[0][2], 0.0, 1e-5);
}
