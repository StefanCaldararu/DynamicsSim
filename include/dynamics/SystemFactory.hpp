#ifndef SYSTEM_FACTORY_H
#define SYSTEM_FACTORY_H

#include "Dynamics.hpp"
#include "Body.hpp"
#include "ODE.hpp"
#include "Integrator.hpp"

namespace Dynamics {

enum class Scenario {
    EarthMoonCR3BP_L4,
    EarthMoonCR3BP_L4_Tadpole,
    TwoBodyMutual,
    OneLargeOneSmall,
    ThreeBodyStable,
    CR3BP_LEO
};

class DynamicsBuilder {
public:
    DynamicsBuilder() = default;

    DynamicsBuilder& withModel(std::unique_ptr<ODE> model);
    DynamicsBuilder& withIntegrator(std::unique_ptr<Integrator> integrator);
    DynamicsBuilder& addBody(const Body& body);

    Dynamics build();

private:
    Dynamics system;
};

class SystemFactory {
public:
    static Dynamics createScenario(Scenario scenario);

    static Dynamics createEarthMoonCR3BP_L4();
    static Dynamics createEarthMoonCR3BP_L4_Tadpole();
    static Dynamics createTwoBodyMutual();
    static Dynamics createOneLargeOneSmall();
    static Dynamics createThreeBodyStable();
    static Dynamics createCR3BPLEO();
};

} // namespace Dynamics

#endif // SYSTEM_FACTORY_H
