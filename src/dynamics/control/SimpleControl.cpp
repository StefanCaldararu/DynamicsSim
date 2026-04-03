#include "dynamics/control/SimpleControl.hpp"
#include <iostream>
namespace Dynamics {

    SimpleControl::SimpleControl(double dv_, double burnDuration_, double burnStartTime_)
    : dv(dv_), burnDuration(burnDuration_), burnStartTime(burnStartTime_) {}

    Eigen::Vector3d SimpleControl::getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {

        if(t >= burnStartTime && t < burnStartTime + burnDuration && velocity.norm() > 1e-5) {
            std::cout << "Applying control at time " << t << "s: dv = " << dv << " m/s" << std::endl;
            return dv * velocity.normalized() / burnDuration;
        } else {
            return Eigen::Vector3d::Zero();
        }
    }
}