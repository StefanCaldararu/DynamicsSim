#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP
#include <vector>
#include <memory>

#include "Body.hpp"
#include "ODE.hpp"
#include "Integrator.hpp"

namespace Dynamics {

class Dynamics {
public:
    Dynamics() = default;

    void setModel(std::unique_ptr<ODE> model);
    void setIntegrator(std::unique_ptr<Integrator> integrator);

    void addBody(const Body& body);
    std::vector<Body>& getBodies();
    const std::vector<Body>& getBodies() const;

    void step(double dt);
    double getTime();

    ODE* getModel() const;

private:
    std::vector<Body> bodies;

    std::unique_ptr<ODE> model;
    std::unique_ptr<Integrator> integrator;
    double time = 0.;
};

}
#endif