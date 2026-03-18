#include "dynamics/GravityModel.hpp"
#include <cmath>

namespace Dynamics {

GravityModel::GravityModel(float G_, const std::vector<Body>& bodies_)
    : G(G_), bodies(bodies_) {}

void GravityModel::derivatives(
    const std::vector<Eigen::Vector3f>& positions,
    const std::vector<Eigen::Vector3f>& velocities,
    std::vector<Eigen::Vector3f>& dpos_dt,
    std::vector<Eigen::Vector3f>& dvel_dt)
{
    int n = positions.size();

    dpos_dt = velocities;
    dvel_dt.resize(n);

    for (int i = 0; i < n; i++)
        dvel_dt[i] = Eigen::Vector3f::Zero();

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {

            Eigen::Vector3f r = positions[j] - positions[i];
            float dist = r.norm();

            if (dist < 1e-5f) continue;

            Eigen::Vector3f force = G * r / std::pow(dist, 3);

            dvel_dt[i] += bodies[j].getMass() * force;
            dvel_dt[j] -= bodies[i].getMass() * force;
        }
    }
}

}