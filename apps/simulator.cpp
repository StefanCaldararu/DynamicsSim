#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "dynamics/SystemFactory.hpp"
#include "vis/Renderer.hpp"

int main() {

    Vis::Renderer renderer = {};

    Dynamics::Dynamics system = Dynamics::SystemFactory::createScenario(Dynamics::Scenario::EarthMoonCR3BP_L4_Tadpole);


    const float dt = 1e-4f;

    while (renderer.isOpen()) {

        renderer.handleEvent();

        system.step(dt);

        renderer.update(system.getBodies());
    }

    return 0;
}