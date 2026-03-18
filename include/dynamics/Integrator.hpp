#pragma once
#include <vector>
#include "Body.hpp"
#include "ODE.hpp"

namespace Dynamics {

class Integrator {
public:
    virtual ~Integrator() = default;

    virtual void step(
        std::vector<Body>& bodies,
        ODE& model,
        float dt
    ) = 0;
};

}