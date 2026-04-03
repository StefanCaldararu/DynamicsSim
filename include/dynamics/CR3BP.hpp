#pragma once
#include "ODE.hpp"

namespace Dynamics {

class CR3BPModel : public ODE {
public:
    CR3BPModel(double mu_);

    void derivatives(
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector3d>& velocities,
        std::vector<Eigen::Vector3d>& dpos_dt,
        std::vector<Eigen::Vector3d>& dvel_dt
    ) override;

private:
    double mu;
};

}