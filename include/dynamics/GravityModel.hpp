#pragma once
#include "ODE.hpp"
#include "Body.hpp"

namespace Dynamics {

class GravityModel : public ODE {
public:
    GravityModel(float G, const std::vector<Body>& bodies);

    void derivatives(
        const std::vector<Eigen::Vector3f>& positions,
        const std::vector<Eigen::Vector3f>& velocities,
        std::vector<Eigen::Vector3f>& dpos_dt,
        std::vector<Eigen::Vector3f>& dvel_dt
    ) override;

private:
    float G;
    const std::vector<Body>& bodies;
};

}