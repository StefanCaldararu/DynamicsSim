#pragma once
#include "Control.hpp"

namespace Dynamics {

class NoControl : public Control {
public:
    Eigen::Vector3d getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) override;
};

}