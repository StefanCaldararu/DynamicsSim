#ifndef BODY_H
#define BODY_H

#include <Eigen/Dense>
namespace Dynamics{


    class Body {
        public:

            Body(Eigen::Vector3d position, Eigen::Vector3d velocity, double mass, double radius);
            
            const Eigen::Vector3d& getPosition() const;
            const Eigen::Vector3d& getVelocity() const;
            double getMass() const;
            double getRadius() const;

            void setPosition(Eigen::Vector3d position);
            void setVelocity(Eigen::Vector3d velocity);





        private:
            Eigen::Vector3d position;
            Eigen::Vector3d velocity;
            double mass;
            double radius;

    };
}

#endif //BODY_H