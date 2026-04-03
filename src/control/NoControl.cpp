#include "control/NoControl.hpp"

namespace Dynamics {

Eigen::Vector3d NoControl::getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    return Eigen::Vector3d::Zero();
}

}
