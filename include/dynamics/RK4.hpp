#ifndef RK4_HPP
#define RK4_HPP
#include "Integrator.hpp"

namespace Dynamics {

class RK4 : public Integrator {
public:
    void step(double t, std::vector<Body>& bodies, ODE& model, double dt) override;
};

}
#endif