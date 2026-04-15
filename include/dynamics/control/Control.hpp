#ifndef CONTROL_H
#define CONTROL_H
#include <Eigen/Dense>

namespace Dynamics {

class Control {
public:
    virtual ~Control() = default;

    virtual Eigen::Vector3d getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity
    ) = 0;

    virtual std::string toString() const = 0;
};

}
#endif