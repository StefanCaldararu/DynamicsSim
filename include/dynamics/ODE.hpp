#pragma once
#include <vector>
#include <Eigen/Dense>

namespace Dynamics {

class ODE {
public:
    virtual ~ODE() = default;

    virtual void derivatives(
        double t,
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector3d>& velocities,
        std::vector<Eigen::Vector3d>& dpos_dt,
        std::vector<Eigen::Vector3d>& dvel_dt
    ) = 0;
};

}