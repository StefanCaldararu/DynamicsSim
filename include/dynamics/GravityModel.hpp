#ifndef GRAVITY_MODEL_HPP
#define GRAVITY_MODEL_HPP
#include "ODE.hpp"
#include "Body.hpp"
#include <memory>

namespace Dynamics {

class GravityModel : public ODE {
public:
    GravityModel(double G, const std::vector<Body>& bodies);

    void derivatives(
        double t,
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector3d>& velocities,
        std::vector<Eigen::Vector3d>& dpos_dt,
        std::vector<Eigen::Vector3d>& dvel_dt
    ) override;

private:
    double G;
    std::vector<Body> bodies;
};

}
#endif