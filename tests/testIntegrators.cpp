#include <gtest/gtest.h>
#include "dynamics/RK4.hpp"
#include "dynamics/Euler.hpp"
#include "dynamics/GravityModel.hpp"
#include "dynamics/Body.hpp"
#include <cmath>

TEST(IntegrationTest, GravityModelSingleStepRK4) {
    double G = 1.0;

    // Massive central body (fixed reference in model)
    Dynamics::Body central(
        Eigen::Vector3d(0,0,0),
        Eigen::Vector3d(0,0,0),
        1000.0,
        1.0
    );

    // Moving test body
    Dynamics::Body body(
        Eigen::Vector3d(1,0,0),
        Eigen::Vector3d(0,1,0),
        1.0,
        1.0
    );

    std::vector<Dynamics::Body> bodies = {body, central};

    Dynamics::GravityModel model(G, bodies);
    Dynamics::RK4 integrator;

    double dt = 0.01;

    Eigen::Vector3d initialPos = bodies[0].getPosition();
    Eigen::Vector3d initialVel = bodies[0].getVelocity();

    integrator.step(0.0, bodies, model, dt);

    Eigen::Vector3d newPos = bodies[0].getPosition();
    Eigen::Vector3d newVel = bodies[0].getVelocity();

    // sanity checks (physics consistency, not exact solution)
    EXPECT_NE(newPos, initialPos);   // moved
    EXPECT_NE(newVel, initialVel);   // velocity changed due to gravity

    // should move toward origin in x-direction (attractive force)
    EXPECT_LT(newVel.x(), 0.0);
}

TEST(IntegrationTest, GravityModelSingleStepEuler) {
    double G = 1.0;

    // Massive central body (fixed reference in model)
    Dynamics::Body central(
        Eigen::Vector3d(0,0,0),
        Eigen::Vector3d(0,0,0),
        1000.0,
        1.0
    );

    // Moving test body
    Dynamics::Body body(
        Eigen::Vector3d(1,0,0),
        Eigen::Vector3d(0,1,0),
        1.0,
        1.0
    );

    std::vector<Dynamics::Body> bodies = {body, central};

    Dynamics::GravityModel model(G, bodies);
    Dynamics::Euler integrator;

    double dt = 0.01;

    Eigen::Vector3d initialPos = bodies[0].getPosition();
    Eigen::Vector3d initialVel = bodies[0].getVelocity();

    integrator.step(0.0, bodies, model, dt);

    Eigen::Vector3d newPos = bodies[0].getPosition();
    Eigen::Vector3d newVel = bodies[0].getVelocity();

    // sanity checks (physics consistency, not exact solution)
    EXPECT_NE(newPos, initialPos);   // moved
    EXPECT_NE(newVel, initialVel);   // velocity changed due to gravity

    // should move toward origin in x-direction (attractive force)
    EXPECT_LT(newVel.x(), 0.0);
}