#include "dynamics/RK4.hpp"

namespace Dynamics {

void RK4::step(std::vector<Body>& bodies, ODE& model, float dt) {

    int n = bodies.size();

    std::vector<Eigen::Vector3f> x(n), v(n);
    for (int i = 0; i < n; i++) {
        x[i] = bodies[i].getPosition();
        v[i] = bodies[i].getVelocity();
    }

    std::vector<Eigen::Vector3f> k1_x, k1_v;
    std::vector<Eigen::Vector3f> k2_x, k2_v;
    std::vector<Eigen::Vector3f> k3_x, k3_v;
    std::vector<Eigen::Vector3f> k4_x, k4_v;

    std::vector<Eigen::Vector3f> xt(n), vt(n);

    model.derivatives(x, v, k1_x, k1_v);

    for (int i = 0; i < n; i++) {
        xt[i] = x[i] + 0.5f * dt * k1_x[i];
        vt[i] = v[i] + 0.5f * dt * k1_v[i];
    }
    model.derivatives(xt, vt, k2_x, k2_v);

    for (int i = 0; i < n; i++) {
        xt[i] = x[i] + 0.5f * dt * k2_x[i];
        vt[i] = v[i] + 0.5f * dt * k2_v[i];
    }
    model.derivatives(xt, vt, k3_x, k3_v);

    for (int i = 0; i < n; i++) {
        xt[i] = x[i] + dt * k3_x[i];
        vt[i] = v[i] + dt * k3_v[i];
    }
    model.derivatives(xt, vt, k4_x, k4_v);

    for (int i = 0; i < n; i++) {

        Eigen::Vector3f x_new =
            x[i] + (dt / 6.0f) *
            (k1_x[i] + 2.0f*k2_x[i] + 2.0f*k3_x[i] + k4_x[i]);

        Eigen::Vector3f v_new =
            v[i] + (dt / 6.0f) *
            (k1_v[i] + 2.0f*k2_v[i] + 2.0f*k3_v[i] + k4_v[i]);

        bodies[i].setPosition(x_new);
        bodies[i].setVelocity(v_new);
    }
}

}