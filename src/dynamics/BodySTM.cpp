#include "dynamics/BodySTM.hpp"

namespace Dynamics {

    BodySTM::BodySTM(Eigen::Vector3d position, Eigen::Vector3d velocity, double mass, double radius)
        : Body(position, velocity, mass, radius) {
            Phi = Eigen::Matrix<double, 6, 6>::Identity();
        }

    Eigen::Matrix<double, 6, 6> BodySTM::getSTM() {
        return Phi;
    }

    std::vector<double> BodySTM::FlattenState() {
        std::vector<double> state(42);
        Eigen::Vector3d pos = getPosition();
        state[0] = pos.x();
        state[1] = pos.y();
        state[2] = pos.z();

        Eigen::Vector3d vel = getVelocity();
        state[3] = vel.x();
        state[4] = vel.y();
        state[5] = vel.z();

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                state[6 + i*6 + j] = Phi(i, j);
            }
        }

        return state;
    }

    void BodySTM::UnpackState() {
        this.setPosition(Eigen::Vector3d(state[0], state[1], state[2]));
        this.setVelocity(Eigen::Vector3d(state[3], state[4], state[5]));

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                Phi(i, j) = state[6 + i*6 + j];
            }
        }
    }
}