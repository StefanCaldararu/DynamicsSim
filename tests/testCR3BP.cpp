#include <gtest/gtest.h>
#include "dynamics/CR3BP.hpp"
#include "dynamics/RK4.hpp"
#include "dynamics/Dynamics.hpp"
#include <vector>
#include <cmath>


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

TEST(CR3BPTest, JacobiConstantMatchesFormula) {
    double mu = 0.01215;
    Dynamics::CR3BPModel model(mu);

    Eigen::Vector3d pos(0.5 - mu, 0.8660254, 0.0);
    Eigen::Vector3d vel(0.0, 0.0, 0.0);

    double r1 = std::sqrt((pos.x() + mu)*(pos.x() + mu) + pos.y()*pos.y() + pos.z()*pos.z());
    double r2 = std::sqrt((pos.x() - (1 - mu))*(pos.x() - (1 - mu)) + pos.y()*pos.y() + pos.z()*pos.z());

    double expectedJacobi = pos.x()*pos.x() + pos.y()*pos.y()
        + 2.0*(1.0 - mu)/r1 + 2.0*mu/r2
        - (vel.x()*vel.x() + vel.y()*vel.y() + vel.z()*vel.z());

    double computedJacobi = model.getJacobiConstant(pos, vel);

    EXPECT_NEAR(computedJacobi, expectedJacobi, 1e-9);
}

TEST(CR3BPTest, JacobiConstantApproxConstThroughIntegration) {
    double mu = 0.01215;
    Dynamics::Dynamics sim;
    sim.addBody(Dynamics::Body({0.5 - mu, 0.8660254, 0.0}, {0.0, 0.0, 0.0}, 1.0, 1.0));

    auto model = std::make_unique<Dynamics::CR3BPModel>(mu);
    sim.setModel(std::move(model));
    sim.setIntegrator(std::make_unique<Dynamics::RK4>());

    double dt = 1e-4;
    int nsteps = 100;

    Eigen::Vector3d pos = sim.getBodies()[0].getPosition();
    Eigen::Vector3d vel = sim.getBodies()[0].getVelocity();
    double c0 = Dynamics::CR3BPModel(mu).getJacobiConstant(pos, vel);

    for (int i = 0; i < nsteps; i++) {
        sim.step(dt);
    }

    Eigen::Vector3d pos1 = sim.getBodies()[0].getPosition();
    Eigen::Vector3d vel1 = sim.getBodies()[0].getVelocity();
    double c1 = Dynamics::CR3BPModel(mu).getJacobiConstant(pos1, vel1);

    EXPECT_NEAR(c1, c0, 1e-4);
}