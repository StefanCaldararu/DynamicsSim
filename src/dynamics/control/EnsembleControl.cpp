#include "dynamics/control/EnsembleControl.hpp"
#include <sstream>

namespace Dynamics {

EnsembleControl::EnsembleControl(std::vector<std::unique_ptr<Control>> controls)
    : controls(std::move(controls)) {}

Eigen::Vector3d EnsembleControl::getAcceleration(double t, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    Eigen::Vector3d totalAccel = Eigen::Vector3d::Zero();
    for (auto& control : controls) {
        totalAccel += control->getAcceleration(t, position, velocity);
    }
    return totalAccel;
}

std::string EnsembleControl::toString() const {
    std::stringstream ss;
    ss << "Ensemble Control - Multiple burns scheduled:\n";
    for (size_t i = 0; i < controls.size(); ++i) {
        ss << "  Control " << (i + 1) << ": " << controls[i]->toString() << "\n";
    }
    return ss.str();
}

}