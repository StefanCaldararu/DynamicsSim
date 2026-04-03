#ifndef NO_CONTROL_H
#define NO_CONTROL_H

#include "Control.hpp"

namespace Dynamics {

class SimpleControl : public Control {
public:
    SimpleControl(double dv_, double burnDuration_, double burnStartTime_);
    Eigen::Vector3d getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) override;
private:
    double dv;
    double burnDuration;
    double burnStartTime;
};

}
#endif