#pragma once
#include "ODE.hpp"

namespace Dynamics {

class CR3BPModel : public ODE {
public:
    CR3BPModel(float mu_);

    void derivatives(
        const std::vector<Eigen::Vector3f>& positions,
        const std::vector<Eigen::Vector3f>& velocities,
        std::vector<Eigen::Vector3f>& dpos_dt,
        std::vector<Eigen::Vector3f>& dvel_dt
    ) override;

private:
    float mu;
};

}