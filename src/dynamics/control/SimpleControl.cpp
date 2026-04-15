#include "dynamics/control/SimpleControl.hpp"
#include <sstream>

namespace Dynamics {

    SimpleControl::SimpleControl(double dv_, double burnDuration_, double burnStartTime_)
    : dv(dv_), burnDuration(burnDuration_), burnStartTime(burnStartTime_) {}

    Eigen::Vector3d SimpleControl::getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {

        if(t >= burnStartTime && t < burnStartTime + burnDuration && velocity.norm() > 1e-5) {
            return dv * velocity.normalized() / burnDuration;
        } else {
            return Eigen::Vector3d::Zero();
        }
    }

    std::string SimpleControl::toString() const {
        std::stringstream ss;
        ss << "Simple Control - Burn scheduled: dv=" << dv << " m/s, duration=" << burnDuration << " s, start=" << burnStartTime << " s";
        return ss.str();
    }
}