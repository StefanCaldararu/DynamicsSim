#include "dynamics/dynamics.hpp"
#include <stdexcept>

namespace Dynamics {

void Dynamics::setModel(std::unique_ptr<ODE> model_) {
    model = std::move(model_);
}

void Dynamics::setIntegrator(std::unique_ptr<Integrator> integrator_) {
    integrator = std::move(integrator_);
}

void Dynamics::addBody(const Body& body) {
    bodies.push_back(body);
}

std::vector<Body>& Dynamics::getBodies() {
    return bodies;
}

const std::vector<Body>& Dynamics::getBodies() const {
    return bodies;
}

double Dynamics::getTime() {
    return time;
}

void Dynamics::step(double dt) {

    if (!model || !integrator) {
        throw std::runtime_error("Dynamics: model or integrator not set");
    }

    integrator->step(bodies, *model, dt);
    time += dt;
}

}