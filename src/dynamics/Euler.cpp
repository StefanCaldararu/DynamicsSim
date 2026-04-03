#include "dynamics/Euler.hpp"

namespace Dynamics {

void Euler::step(double t, std::vector<Body>& bodies, ODE& model, double dt) {

    int n = bodies.size();

    std::vector<Eigen::Vector3d> x(n), v(n);
    for (int i = 0; i < n; i++) {
        x[i] = bodies[i].getPosition();
        v[i] = bodies[i].getVelocity();
    }

    std::vector<Eigen::Vector3d> dx, dv;
    model.derivatives(t, x, v, dx, dv);

    for (int i = 0; i < n; i++) {
        bodies[i].setPosition(x[i] + dt * dx[i]);
        bodies[i].setVelocity(v[i] + dt * dv[i]);
    }
}

}