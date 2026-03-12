#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "dynamics/body.hpp"
#include "dynamics/dynamics.hpp"
#include "vis/renderer.hpp"

int main() {

    Vis::Renderer renderer = {};

    // Create dynamics system
    Dynamics::Dynamics system(1.0f);

    // Create two bodies
    Dynamics::Body body1(
        Eigen::Vector3f(400.f, 300.f, 0.f),   // position
        Eigen::Vector3f(0.f, 0.f, 0.f),       // velocity
        1000.f,                               // mass
        12.f                                  // radius
    );

    Dynamics::Body body2(
        Eigen::Vector3f(500.f, 300.f, 0.f),
        Eigen::Vector3f(0.f, 3.2f, 0.f),
        1.f,
        6.f
    );

    system.addBody(body1);
    system.addBody(body2);

    const float dt = 0.01f;

    while (renderer.isOpen()) {

        renderer.handleEvent();

        // Update physics
        system.update(dt);

        renderer.update(system.getBodies());
    }

    return 0;
}