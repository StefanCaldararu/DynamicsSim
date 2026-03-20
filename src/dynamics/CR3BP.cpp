#include "dynamics/CR3BP.hpp"
#include <cmath>

namespace Dynamics {

CR3BPModel::CR3BPModel(float mu_)
    : mu(mu_)
{}

void CR3BPModel::derivatives(
    const std::vector<Eigen::Vector3f>& positions,
    const std::vector<Eigen::Vector3f>& velocities,
    std::vector<Eigen::Vector3f>& dpos_dt,
    std::vector<Eigen::Vector3f>& dvel_dt
) {

    int n = positions.size();

    dpos_dt.resize(n);
    dvel_dt.resize(n);

    for (int i = 0; i < n; i++) {
        dpos_dt[i] = Eigen::Vector3f::Zero();
        dvel_dt[i] = Eigen::Vector3f::Zero();
    }

    if (n < 1) return;

    const auto& r = positions[0];
    const auto& v = velocities[0];

    float x = r.x();
    float y = r.y();
    float z = r.z();

    float vx = v.x();
    float vy = v.y();
    float vz = v.z();

    float r1 = std::sqrt((x + mu)*(x + mu) + y*y + z*z);
    float r2 = std::sqrt((x - (1 - mu))*(x - (1 - mu)) + y*y + z*z);

    dpos_dt[0] = v;

    float ax =
        2.0f * vy + x
        - (1 - mu)*(x + mu)/std::pow(r1, 3)
        - mu*(x - (1 - mu))/std::pow(r2, 3);

    float ay =
        -2.0f * vx + y
        - (1 - mu)*y/std::pow(r1, 3)
        - mu*y/std::pow(r2, 3);

    float az =
        - (1 - mu)*z/std::pow(r1, 3)
        - mu*z/std::pow(r2, 3);

    dvel_dt[0] = Eigen::Vector3f(ax, ay, az);

}

}