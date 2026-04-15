#include "dynamics/STMCR3BP.hpp"

namespace Dynamics {

    STMCR3BPModel::STMCR3BPModel(double mu_, std::unique_ptr<Control> control_)
        : CR3BPModel(mu_, std::move(control_))
    {}

    void CR3BPModel::derivatives42(double t, const std::vector<Eigen::Vector3d>& positions, const std::vector<Eigen::Vector3d>& velocities, std::vector<Eigen::Vector3d>& dpos_dt, std::vector<Eigen::Vector3d>& dvel_dt, Eigen::Matrix<double, 6, 6>& dPhi_dt) {
        self.derivatives(t, positions, velocities, dpos_dt, dvel_dt);

        Eigen::Matrix<double,6,6> A = computeJacobian(positions[0], velocities[0]);

        dPhi_dt = A * dPhi_dt;
    }


    Eigen::Matrix<double, 6, 6> STMCR3BPModel::computeJacobian(const Eigen::Vector3d& position) const {
        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();

        Eigen::Matrix<double,6,6> A;
        A.setZero();

        double x = position.x();
        double y = position.y();
        double z = position.z();

        double r1 = sqrt((x+mu)*(x+mu) + y*y + z*z);
        double r2 = sqrt((x-(1-mu))*(x-(1-mu)) + y*y + z*z);

        double r13 = pow(r1,3);
        double r15 = pow(r1,5);
        double r23 = pow(r2,3);
        double r25 = pow(r2,5);

        double Uxx =
            1
            - (1-mu)*(1/r13 - 3*(x+mu)*(x+mu)/r15)
            - mu*(1/r23 - 3*(x-(1-mu))*(x-(1-mu))/r25);

        double Uyy =
            1
            - (1-mu)*(1/r13 - 3*y*y/r15)
            - mu*(1/r23 - 3*y*y/r25);

        double Uzz =
            - (1-mu)*(1/r13 - 3*z*z/r15)
            - mu*(1/r23 - 3*z*z/r25);

        double Uxy =
            3*y*((1-mu)*(x+mu)/r15 + mu*(x-(1-mu))/r25);

        double Uxz =
            3*z*((1-mu)*(x+mu)/r15 + mu*(x-(1-mu))/r25);

        double Uyz =
            3*y*z*((1-mu)/r15 + mu/r25);

        // Fill matrix
        A(0,3) = 1;
        A(1,4) = 1;
        A(2,5) = 1;

        A(3,0) = Uxx;
        A(3,1) = Uxy;
        A(3,2) = Uxz;
        A(3,4) = 2;

        A(4,0) = Uxy;
        A(4,1) = Uyy;
        A(4,2) = Uyz;
        A(4,3) = -2;

        A(5,0) = Uxz;
        A(5,1) = Uyz;
        A(5,2) = Uzz;

        return A;
    }

}