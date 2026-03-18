#pragma once
#include <vector>
#include <Eigen/Dense>

namespace Dynamics {

class ODE {
public:
    virtual ~ODE() = default;

    virtual void derivatives(
        const std::vector<Eigen::Vector3f>& positions,
        const std::vector<Eigen::Vector3f>& velocities,
        std::vector<Eigen::Vector3f>& dpos_dt,
        std::vector<Eigen::Vector3f>& dvel_dt
    ) = 0;
};

}