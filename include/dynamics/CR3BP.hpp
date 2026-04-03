#pragma once
#include "ODE.hpp"
#include "control/Control.hpp"

namespace Dynamics {

class CR3BPModel : public ODE {
public:
    CR3BPModel(double mu_, Control* control_ = nullptr);

    void derivatives(
        double t,
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector3d>& velocities,
        std::vector<Eigen::Vector3d>& dpos_dt,
        std::vector<Eigen::Vector3d>& dvel_dt
    ) override;

private:
    double mu;
    Control* control;
};

}