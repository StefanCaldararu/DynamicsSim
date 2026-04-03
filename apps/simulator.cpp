#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "dynamics/SystemFactory.hpp"
#include "vis/Renderer.hpp"

int main() {

    Vis::Renderer renderer = {};

    Dynamics::Dynamics system = Dynamics::SystemFactory::createScenario(Dynamics::Scenario::CR3BP_LEO);


    const double dt = 1e-4;

    while (renderer.isOpen()) {

        renderer.handleEvent();

        system.step(dt);

        renderer.update(system.getBodies(), system.getTime());
    }

    return 0;
}