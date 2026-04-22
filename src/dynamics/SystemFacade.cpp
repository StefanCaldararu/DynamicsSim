#include "dynamics/SystemFacade.hpp"
#include "dynamics/CR3BP.hpp"
#include "dynamics/GravityModel.hpp"
#include "dynamics/RK4.hpp"
#include "dynamics/Euler.hpp"

namespace Dynamics {

DynamicsBuilder& DynamicsBuilder::withModel(std::unique_ptr<ODE> model) {
    system.setModel(std::move(model));
    return *this;
}

DynamicsBuilder& DynamicsBuilder::withIntegrator(std::unique_ptr<Integrator> integrator) {
    system.setIntegrator(std::move(integrator));
    return *this;
}

DynamicsBuilder& DynamicsBuilder::addBody(const Body& body) {
    system.addBody(body);
    return *this;
}

std::vector<Body> DynamicsBuilder::getBodies(){
    return system.getBodies();
}

Dynamics DynamicsBuilder::build() {
    return std::move(system);
}

Dynamics SystemFacade::createScenario(Scenario scenario) {
    switch (scenario) {
        case Scenario::EarthMoonCR3BP_L4:
            return createEarthMoonCR3BP_L4();
        case Scenario::EarthMoonCR3BP_L4_Tadpole:
            return createEarthMoonCR3BP_L4_Tadpole();
        case Scenario::TwoBodyMutual:
            return createTwoBodyMutual();
        case Scenario::OneLargeOneSmall:
            return createOneLargeOneSmall();
        case Scenario::ThreeBodyStable:
            return createThreeBodyStable();
        case Scenario::CR3BP_LEO:
            return createCR3BPLEO();
        default:
            return createEarthMoonCR3BP_L4();
    }
}

Dynamics SystemFacade::createEarthMoonCR3BP_L4() {
    const double mu = 0.01215;
    DynamicsBuilder builder;
    Body spacecraft = Body({0.5 - mu, 0.8660254, 0.0}, {0,0,0}, 0.0, 1.0);
    Body earth = Body({-mu, 0.0, 0.0}, {0,0,0}, 1.0, 10.0);
    Body moon = Body({1.0 - mu, 0.0, 0.0}, {0,0,0}, mu, 5.0);

    builder.addBody(spacecraft).addBody(earth).addBody(moon).withModel(std::make_unique<CR3BPModel>(mu)).withIntegrator(std::make_unique<RK4>());

    return builder.build();
}


Dynamics SystemFacade::createEarthMoonCR3BP_L4_Tadpole() {
    const double mu = 0.01215;
    DynamicsBuilder builder;
    // Short period L4 orbit grabbed from ID 768
    // https://ssd.jpl.nasa.gov/tools/periodic_orbits.html
    Body spacecraft = Body({4.8784941344943100e-1, 1.3592776304398071, 0.}, {7.3994557499431435e-1, -3.6396916907329402e-1, 0.}, 0., 1.0);
    Body earth = Body({-mu, 0.0, 0.0}, {0,0,0}, 1.0, 10.0);
    Body moon = Body({1.0 - mu, 0.0, 0.0}, {0,0,0}, mu, 5.0);

    builder.addBody(spacecraft).addBody(earth).addBody(moon).withModel(std::make_unique<CR3BPModel>(mu)).withIntegrator(std::make_unique<RK4>());

    return builder.build();
}

Dynamics SystemFacade::createTwoBodyMutual() {
    DynamicsBuilder builder;

    Body body1 = Body({-0.5, 0.0, 0.0}, {0.0, 1.85, 0.0}, 10.0, 1.0);
    Body body2 = Body({0.5, 0.0, 0.0}, {0.0, -1.85, 0.0}, 10.0, 1.0); 

    builder.addBody(body1).addBody(body2).withModel(std::make_unique<GravityModel>(1.0, builder.getBodies())).withIntegrator(std::make_unique<RK4>());

    return builder.build();
}

Dynamics SystemFacade::createOneLargeOneSmall() {
    DynamicsBuilder builder;

    Body large = Body({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1000.0, 5.0);
    Body small = Body({5.0, 0.0, 0.0}, {0.0, 9.5, 0.0}, 1.0, 0.1);

    builder.addBody(large).addBody(small).withModel(std::make_unique<GravityModel>(1.0, builder.getBodies())).withIntegrator(std::make_unique<Euler>());

    return builder.build();
}

Dynamics SystemFacade::createThreeBodyStable() {
    DynamicsBuilder builder;

    Body body1 = Body({0.0, -1.0, 0.0}, {0.58, 0.0, 0.0}, 1.0, 1.0);
    Body body2 = Body({0.8660254, 0.5, 0.0}, {-0.29, 0.5, 0.0}, 1.0, 1.0);
    Body body3 = Body({-0.8660254, 0.5, 0.0}, {-0.29, -0.5, 0.0}, 1.0, 1.0);

    builder.addBody(body1).addBody(body2).addBody(body3).withModel(std::make_unique<GravityModel>(1.0, builder.getBodies())).withIntegrator(std::make_unique<RK4>());

    return builder.build();
}

Dynamics SystemFacade::createCR3BPLEO() {
    const double mu = 0.01215;
    DynamicsBuilder builder;

    double r_leo = 0.0176;
    double x = -mu + r_leo;
    double y = 0.0;
    double v_inertial = std::sqrt((1.0 - mu) / r_leo);
    double vy = v_inertial - x;

    Body spacecraft = Body({x, y, 0.0}, {0.0, vy, 0.0}, 0.0, 1.0);
    Body earth =  Body({-mu, 0.0, 0.0}, {0,0,0}, 1.0, 10.0);
    Body moon = Body({1.0 - mu, 0.0, 0.0}, {0,0,0}, mu, 5.0); 
    builder.addBody(spacecraft).addBody(earth).addBody(moon).withModel(std::make_unique<CR3BPModel>(mu)).withIntegrator(std::make_unique<RK4>());

    return builder.build();
}

}
