#include "dynamics/SystemFactory.hpp"
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

Dynamics DynamicsBuilder::build() {
    return std::move(system);
}

Dynamics SystemFactory::createScenario(Scenario scenario) {
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

Dynamics SystemFactory::createEarthMoonCR3BP_L4() {
    const float mu = 0.01215f;
    DynamicsBuilder builder;

    builder.addBody(Body({0.5f - mu, 0.8660254f, 0.0f}, {0,0,0}, 0.0f, 1.0f));
    builder.addBody(Body({-mu, 0.0f, 0.0f}, {0,0,0}, 1.0f, 10.0f));
    builder.addBody(Body({1.0f - mu, 0.0f, 0.0f}, {0,0,0}, mu, 5.0f));

    builder.withModel(std::make_unique<CR3BPModel>(mu));
    builder.withIntegrator(std::make_unique<RK4>());

    return builder.build();
}

Dynamics SystemFactory::createEarthMoonCR3BP_L4_Tadpole() {
    const float mu = 0.01215f;
    DynamicsBuilder builder;

    // Spacecraft in tadpole orbit around L4 with initial velocity perturbation
    builder.addBody(Body({0.5f - mu, 0.8660254f, 0.0f}, {-0.22f, 0.05f, 0.0f}, 0.0f, 2.0f));
    builder.addBody(Body({-mu, 0.0f, 0.0f}, {0,0,0}, 1.0f, 10.0f));
    builder.addBody(Body({1.0f - mu, 0.0f, 0.0f}, {0,0,0}, mu, 5.0f));

    builder.withModel(std::make_unique<CR3BPModel>(mu));
    builder.withIntegrator(std::make_unique<RK4>());

    return builder.build();
}

Dynamics SystemFactory::createTwoBodyMutual() {
    Dynamics system;

    system.addBody(Body({-0.5f, 0.0f, 0.0f}, {0.0f, 0.65f, 0.0f}, 10.0f, 1.0f));
    system.addBody(Body({0.5f, 0.0f, 0.0f}, {0.0f, -0.65f, 0.0f}, 10.0f, 1.0f));

    system.setModel(std::make_unique<GravityModel>(1.0f, system.getBodies()));
    system.setIntegrator(std::make_unique<RK4>());

    return system;
}

Dynamics SystemFactory::createOneLargeOneSmall() {
    Dynamics system;

    system.addBody(Body({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 1000.0f, 10.0f));
    system.addBody(Body({5.0f, 0.0f, 0.0f}, {0.0f, 4.5f, 0.0f}, 1.0f, 0.5f));

    system.setModel(std::make_unique<GravityModel>(1.0f, system.getBodies()));
    system.setIntegrator(std::make_unique<Euler>());

    return system;
}

Dynamics SystemFactory::createThreeBodyStable() {
    Dynamics system;

    system.addBody(Body({0.0f, -1.0f, 0.0f}, {0.58f, 0.0f, 0.0f}, 1.0f, 1.0f));
    system.addBody(Body({0.8660254f, 0.5f, 0.0f}, {-0.29f, 0.5f, 0.0f}, 1.0f, 1.0f));
    system.addBody(Body({-0.8660254f, 0.5f, 0.0f}, {-0.29f, -0.5f, 0.0f}, 1.0f, 1.0f));

    system.setModel(std::make_unique<GravityModel>(1.0f, system.getBodies()));
    system.setIntegrator(std::make_unique<RK4>());

    return system;
}

Dynamics SystemFactory::createCR3BPLEO() {
    const float mu = 0.01215f;
    DynamicsBuilder builder;

    builder.addBody(Body({0.01f, 6770.0f, 0.0f}, {7.8f, 0.0f, 0.0f}, 0.0f, 1.0f));
    builder.addBody(Body({-mu, 0.0f, 0.0f}, {0,0,0}, 1.0f, 10.0f));
    builder.addBody(Body({1.0f - mu, 0.0f, 0.0f}, {0,0,0}, mu, 5.0f));

    builder.withModel(std::make_unique<CR3BPModel>(mu));
    builder.withIntegrator(std::make_unique<RK4>());

    return builder.build();
}

} // namespace Dynamics
