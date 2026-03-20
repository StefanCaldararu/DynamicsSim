#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "dynamics/Body.hpp"
#include "dynamics/Dynamics.hpp"
#include "dynamics/GravityModel.hpp"
#include "dynamics/RK4.hpp"
#include "dynamics/Euler.hpp"
#include "vis/Renderer.hpp"
#include "dynamics/CR3BP.hpp"
int main() {

    Vis::Renderer renderer = {};

    Dynamics::Dynamics system;

    float m1 = 1.0f;
    float m2 = 0.01215f;

    Dynamics::Body primary1({-0.01215f, 0.0f, 0.0f}, {0,0,0}, m1, 10.0f);
    Dynamics::Body primary2({1.0f - 0.01215f, 0.0f, 0.0f}, {0,0,0}, m2, 5.0f);

    Dynamics::Body spacecraft(
        {0.48785f, 0.866f, 0.0f},
        {0.0f, -0.05f, 0.0f},
        0.0f,
        2.0f
    );

    // Dynamics::Body spacecraft(
    //     {0.5f - m2, 0.8660254f, 0.0f},  
    //     {0.0f, 0.0f, 0.0f},             
    //     0.0f,
    //     2.0f
    // );

    // Dynamics::Body spacecraft(
    //     {0.2f, 0.0f, 0.0f},    
    //     {0.0f, 0.25f, 0.0f},  
    //     0.0f,
    //     2.0f
    // );

    system.addBody(spacecraft);
    system.addBody(primary1);
    system.addBody(primary2);
    system.setModel(std::make_unique<Dynamics::CR3BPModel>(0.01215f));
    // system.setModel(std::make_unique<Dynamics::GravityModel>(1.f, system.getBodies()));
    system.setIntegrator(std::make_unique<Dynamics::RK4>());

    const float dt = 1e-4f;

    while (renderer.isOpen()) {

        renderer.handleEvent();

        system.step(dt);

        renderer.update(system.getBodies());
    }

    return 0;
}