#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "dynamics/SystemFacade.hpp"
#include "dynamics/CR3BP.hpp"
#include "vis/Renderer.hpp"

int main() {

    Vis::Renderer renderer = {};

    Dynamics::Dynamics system = Dynamics::SystemFacade::createScenario(Dynamics::Scenario::EarthMoonCR3BP_L4_Tadpole);

    const double dt = 1e-3;

    while (renderer.isOpen()) {

        renderer.handleEvent();

        system.step(dt);

        renderer.update(system.getBodies(), system.getTime());
    }

    return 0;
}