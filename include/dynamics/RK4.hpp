#pragma once
#include "Integrator.hpp"

namespace Dynamics {

class RK4 : public Integrator {
public:
    void step(std::vector<Body>& bodies, ODE& model, float dt) override;
};

}