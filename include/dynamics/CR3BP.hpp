#pragma once
#include "ODE.hpp"
#include "control/Control.hpp"
#include <memory>

namespace Dynamics {

class CR3BPModel : public ODE {
public:
    CR3BPModel(double mu_, std::unique_ptr<Control> control_ = nullptr);

    void derivatives(
        double t,
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector3d>& velocities,
        std::vector<Eigen::Vector3d>& dpos_dt,
        std::vector<Eigen::Vector3d>& dvel_dt
    ) override;

    double getJacobiConstant(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) const;

private:
    double mu;
    std::unique_ptr<Control> control;
};

}