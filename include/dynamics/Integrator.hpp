#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP
#include <vector>
#include "Body.hpp"
#include "ODE.hpp"

namespace Dynamics {

class Integrator {
public:
    virtual ~Integrator() = default;

    virtual void step(
        double t,
        std::vector<Body>& bodies,
        ODE& model,
        double dt
    ) = 0;
};

}
#endif