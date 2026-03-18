#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "dynamics/Body.hpp"
#include "dynamics/Dynamics.hpp"
#include "dynamics/GravityModel.hpp"
#include "dynamics/RK4.hpp"
#include "dynamics/Euler.hpp"
#include "vis/Renderer.hpp"

int main() {

    Vis::Renderer renderer = {};

    Dynamics::Dynamics system;


    Dynamics::Body body1(
        Eigen::Vector3f(400.f, 300.f, 0.f),
        Eigen::Vector3f(0.f, 0.f, 0.f),
        1000.f,
        12.f
    );

    Dynamics::Body body2(
        Eigen::Vector3f(500.f, 300.f, 0.f),
        Eigen::Vector3f(0.f, 3.2f, 0.f),
        10.f,
        6.f
    );

    // Dynamics::Body body3(
    //     Eigen::Vector3f(300.f, 300.f, 0.f),
    //     Eigen::Vector3f(0.f, -3.2f, 0.f),
    //     7.f,
    //     5.f        
    // );

    system.addBody(body1);
    system.addBody(body2);
    system.addBody(body3);
    system.setModel(std::make_unique<Dynamics::GravityModel>(1.0f, system.getBodies()));
    system.setIntegrator(std::make_unique<Dynamics::Euler>());

    const float dt = 0.01f;

    while (renderer.isOpen()) {

        renderer.handleEvent();

        system.step(dt);

        renderer.update(system.getBodies());
    }

    return 0;
}