#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "dynamics/SystemFactory.hpp"
#include "vis/Renderer.hpp"

int main() {

    Vis::Renderer renderer = {};

    Dynamics::Dynamics system = Dynamics::SystemFactory::createScenario(Dynamics::Scenario::EarthMoonCR3BP_L4_Tadpole);


    const double dt = 1e-5;
    int stepcount = 1000;
    while (renderer.isOpen()) {
        renderer.handleEvent();
        system.step(dt);
        double jc = system.getJacobiConstant();
        if(stepcount == 1000){
            stepcount = 0;
            renderer.update(system.getBodies(), system.getTime(), jc);
        }
        stepcount++;
    }

    return 0;
}