#include "dynamics/GravityModel.hpp"
#include <cmath>

namespace Dynamics {

GravityModel::GravityModel(double G_, const std::vector<Body>& bodies_, std::unique_ptr<Control> control_)
    : G(G_), bodies(bodies_), control(std::move(control_)) {}

void GravityModel::derivatives(
    double t,
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Eigen::Vector3d>& velocities,
    std::vector<Eigen::Vector3d>& dpos_dt,
    std::vector<Eigen::Vector3d>& dvel_dt)
{
    int n = positions.size();

    dpos_dt = velocities;
    dvel_dt.resize(n);

    for (int i = 0; i < n; i++)
        dvel_dt[i] = Eigen::Vector3d::Zero();

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {

            Eigen::Vector3d r = positions[j] - positions[i];
            double dist = r.norm();

            if (dist < 1e-5) continue;

            Eigen::Vector3d force = G * r / std::pow(dist, 3);

            dvel_dt[i] += bodies[j].getMass() * force;
            dvel_dt[j] -= bodies[i].getMass() * force;
        }
    }

    if (control && n > 0) {
        dvel_dt[0] += control->getAcceleration(t, positions[0], velocities[0]);
    }
}

}