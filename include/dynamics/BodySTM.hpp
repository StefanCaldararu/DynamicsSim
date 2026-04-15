#ifndef BODY_STM_H
#define BODY_STM_H

#include <Eigen/Dense>
namespace Dynamics{


    class BodySTM : public Body {
        public:

            BodySTM(Eigen::Vector3d position, Eigen::Vector3d velocity, double mass, double radius);

            Eigen::Matrix<double, 6, 6> getSTM();
            std::vector<double> FlattenState();
            void UnpackState();
        private:
            Eigen::Matrix<double, 6, 6> Phi;

    };
}

#endif //BODY_H