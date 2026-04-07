#ifndef BODY_H
#define BODY_H

#include <Eigen/Dense>
namespace Dynamics{


    class Body {
        public:

            Body(Eigen::Vector3d position_, Eigen::Vector3d velocity_, double mass_, double radius_);
            Body(std::vector<double> state_, double mass_, double radius_);
            const Eigen::Vector3d getPosition() const;
            const Eigen::Vector3d getVelocity() const;
            double getMass() const;
            double getRadius() const;
            std::vector<double> getState();

            void setPosition(Eigen::Vector3d position);
            void setVelocity(Eigen::Vector3d velocity);
            void setState(std::vector<double> state);





        private:
            //Assumed <x, y, z, dx, dy, dz, ANYTHING ELSE>
            std::vector<double> state; 
            double mass;
            double radius;

    };
}

#endif //BODY_H