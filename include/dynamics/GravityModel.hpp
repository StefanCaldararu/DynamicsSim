#pragma once
#include "ODE.hpp"
#include "Body.hpp"
#include "control/Control.hpp"
#include <memory>

namespace Dynamics {

class GravityModel : public ODE {
public:
    GravityModel(double G, const std::vector<Body>& bodies, std::unique_ptr<Control> control_ = nullptr);

    void derivatives(
        double t,
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector3d>& velocities,
        std::vector<Eigen::Vector3d>& dpos_dt,
        std::vector<Eigen::Vector3d>& dvel_dt
    ) override;

double getJacobiConstant(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) const override;

private:
    double G;
    const std::vector<Body>& bodies;
    std::unique_ptr<Control> control;
};

}