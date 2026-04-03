#include <gtest/gtest.h>
#include "dynamics/GravityModel.hpp"
#include "dynamics/Body.hpp"
#include <vector>


TEST(GravityModelTest, posDerivativeEqualVel) {

    Eigen::Vector3d pos(1,0,0);
    Eigen::Vector3d vel(1,2,3);
    std::vector<Eigen::Vector3d> positions = {pos};
    std::vector<Eigen::Vector3d> velocities = {vel};

    Dynamics::Body b(pos, vel, 1.0, 1.0);
    Dynamics::GravityModel model(1.0, {b});

    std::vector<Eigen::Vector3d> dpos_dt;
    std::vector<Eigen::Vector3d> dvel_dt;

    model.derivatives(positions, velocities, dpos_dt, dvel_dt);


    for(int i = 0; i < 3; i++){
        EXPECT_DOUBLE_EQ(dpos_dt[0][i], velocities[0][i]);
    }
}