#include "dynamics/RK4.hpp"

namespace Dynamics {

void RK4::step(std::vector<Body>& bodies, ODE& model, double dt) {

    int n = bodies.size();

    std::vector<Eigen::Vector3d> x(n), v(n);
    for (int i = 0; i < n; i++) {
        x[i] = bodies[i].getPosition();
        v[i] = bodies[i].getVelocity();
    }

    std::vector<Eigen::Vector3d> k1_x, k1_v;
    std::vector<Eigen::Vector3d> k2_x, k2_v;
    std::vector<Eigen::Vector3d> k3_x, k3_v;
    std::vector<Eigen::Vector3d> k4_x, k4_v;

    std::vector<Eigen::Vector3d> xt(n), vt(n);

    model.derivatives(x, v, k1_x, k1_v);

    for (int i = 0; i < n; i++) {
        xt[i] = x[i] + 0.5 * dt * k1_x[i];
        vt[i] = v[i] + 0.5 * dt * k1_v[i];
    }
    model.derivatives(xt, vt, k2_x, k2_v);

    for (int i = 0; i < n; i++) {
        xt[i] = x[i] + 0.5 * dt * k2_x[i];
        vt[i] = v[i] + 0.5 * dt * k2_v[i];
    }
    model.derivatives(xt, vt, k3_x, k3_v);

    for (int i = 0; i < n; i++) {
        xt[i] = x[i] + dt * k3_x[i];
        vt[i] = v[i] + dt * k3_v[i];
    }
    model.derivatives(xt, vt, k4_x, k4_v);

    for (int i = 0; i < n; i++) {

        Eigen::Vector3d x_new =
            x[i] + (dt / 6.0) *
            (k1_x[i] + 2.0*k2_x[i] + 2.0*k3_x[i] + k4_x[i]);

        Eigen::Vector3d v_new =
            v[i] + (dt / 6.0) *
            (k1_v[i] + 2.0*k2_v[i] + 2.0*k3_v[i] + k4_v[i]);

        bodies[i].setPosition(x_new);
        bodies[i].setVelocity(v_new);
    }
}

}