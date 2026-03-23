#include <gtest/gtest.h>
#include "dynamics/GravityModel.hpp"
#include "dynamics/Body.hpp"
#include <vector>


TEST(GravityModelTest, posDerivativeEqualVel) {

    Eigen::Vector3f pos(1,0,0);
    Eigen::Vector3f vel(1,2,3);
    std::vector<Eigen::Vector3f> positions = {pos};
    std::vector<Eigen::Vector3f> velocities = {vel};

    Dynamics::Body b(pos, vel, 1.f, 1.f);
    Dynamics::GravityModel model(1.f, {b});

    std::vector<Eigen::Vector3f> dpos_dt;
    std::vector<Eigen::Vector3f> dvel_dt;

    model.derivatives(positions, velocities, dpos_dt, dvel_dt);


    for(int i = 0; i < 3; i++){
        EXPECT_FLOAT_EQ(dpos_dt[0][i], velocities[0][i]);
    }
}