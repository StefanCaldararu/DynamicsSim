#include "dynamics/CR3BP.hpp"
#include <cmath>

namespace Dynamics {

CR3BPModel::CR3BPModel(double mu_, Control* control_)
    : mu(mu_), control(control_)
{}

void CR3BPModel::derivatives(
    double t,
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Eigen::Vector3d>& velocities,
    std::vector<Eigen::Vector3d>& dpos_dt,
    std::vector<Eigen::Vector3d>& dvel_dt
) {

    int n = positions.size();

    dpos_dt.resize(n);
    dvel_dt.resize(n);

    for (int i = 0; i < n; i++) {
        dpos_dt[i] = Eigen::Vector3d::Zero();
        dvel_dt[i] = Eigen::Vector3d::Zero();
    }

    if (n < 1) return;

    const auto& r = positions[0];
    const auto& v = velocities[0];

    double x = r.x();
    double y = r.y();
    double z = r.z();

    double vx = v.x();
    double vy = v.y();
    double vz = v.z();

    double r1 = std::sqrt((x + mu)*(x + mu) + y*y + z*z);
    double r2 = std::sqrt((x - (1 - mu))*(x - (1 - mu)) + y*y + z*z);

    dpos_dt[0] = v;

    double ax =
        2.0 * vy + x
        - (1 - mu)*(x + mu)/std::pow(r1, 3)
        - mu*(x - (1 - mu))/std::pow(r2, 3);

    double ay =
        -2.0 * vx + y
        - (1 - mu)*y/std::pow(r1, 3)
        - mu*y/std::pow(r2, 3);

    double az =
        - (1 - mu)*z/std::pow(r1, 3)
        - mu*z/std::pow(r2, 3);

    dvel_dt[0] = Eigen::Vector3d(ax, ay, az);

    if (control) {
        dvel_dt[0] += control->getAcceleration(t, r, v);
    }

}

double CR3BPModel::getJacobiConstant(const Eigen::Vector3d& position,
                                     const Eigen::Vector3d& velocity) const {
    double x = position.x();
    double y = position.y();
    double z = position.z();

    double vx = velocity.x();
    double vy = velocity.y();
    double vz = velocity.z();

    double r1 = std::sqrt((x + mu)*(x + mu) + y*y + z*z);
    double r2 = std::sqrt((x - (1 - mu))*(x - (1 - mu)) + y*y + z*z);

    double potential = 0.5 * (x*x + y*y) + (1 - mu) / r1 + mu / r2;
    double kinetic = 0.5 * (vx*vx + vy*vy + vz*vz);

    return 2.0 * potential - (vx*vx + vy*vy + vz*vz);
}

}