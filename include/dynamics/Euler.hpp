#pragma once
#include "Integrator.hpp"

namespace Dynamics {

class Euler : public Integrator {
public:
    void step(std::vector<Body>& bodies, ODE& model, float dt) override;
};

}