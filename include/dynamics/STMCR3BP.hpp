
#ifndef STMCR3BP_HPP
#define STMCR3BP_HPP

#include "dynamics/CR3BP.hpp"

namespace Dynamics {

    class STMCR3BPModel : public CR3BPModel {
        public:
            STMCR3BPModel(double mu_);

            void CR3BPModel::derivatives42(double t, const std::vector<Eigen::Vector3d>& positions, const std::vector<Eigen::Vector3d>& velocities, std::vector<Eigen::Vector3d>& dpos_dt, std::vector<Eigen::Vector3d>& dvel_dt, Eigen::Matrix<double, 6, 6>& dPhi_dt);


            Eigen::Matrix<double, 6, 6> computeJacobian(const Eigen::Vector3d& position) const;
    };
}
#endif