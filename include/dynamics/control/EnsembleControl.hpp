#ifndef ENSEMBLE_CONTROL_H 
#define ENSEMBLE_CONTROL_H

#include "Control.hpp"
#include <vector>
#include <memory>

namespace Dynamics {

class EnsembleControl : public Control {
public:
    EnsembleControl(std::vector<std::unique_ptr<Control>> controls);
    Eigen::Vector3d getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) override;
    std::string toString() const override;
private:
    std::vector<std::unique_ptr<Control>> controls;
};

}
#endif