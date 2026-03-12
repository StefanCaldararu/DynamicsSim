#include "dynamics/Dynamics.hpp"
#include <cmath>

namespace Dynamics {

Dynamics::Dynamics(float G_)
    : G(G_)
{}

void Dynamics::addBody(const Body& body) {
    bodies.push_back(body);
}

const std::vector<Body>& Dynamics::getBodies() const {
    return bodies;
}

void Dynamics::computeAccelerations(std::vector<Eigen::Vector3f>& accels) {

    int n = bodies.size();
    accels.resize(n);

    for (int i = 0; i < n; i++)
        accels[i] = Eigen::Vector3f::Zero();

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {

            Eigen::Vector3f r =
                bodies[j].getPosition() -
                bodies[i].getPosition();

            float dist = r.norm();

            if (dist < 1e-3f)
                continue;

            Eigen::Vector3f force =
                G * r / std::pow(dist, 3);

            accels[i] += bodies[j].getMass() * force;
            accels[j] -= bodies[i].getMass() * force;
        }
    }
}

void Dynamics::update(float dt) {

    std::vector<Eigen::Vector3f> accels;

    computeAccelerations(accels);

    for (size_t i = 0; i < bodies.size(); i++) {

        Eigen::Vector3f v =
            bodies[i].getVelocity();

        Eigen::Vector3f x =
            bodies[i].getPosition();

        v += accels[i] * dt;
        x += v * dt;

        bodies[i].setVelocity(v);
        bodies[i].setPosition(x);
    }
}

}