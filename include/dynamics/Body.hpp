#ifndef BODY_H
#define BODY_H

#include <Eigen/Dense>
namespace Dynamics{


    class Body {
        public:

            Body(Eigen::Vector3f position, Eigen::Vector3f velocity, float mass, float radius);
            
            const Eigen::Vector3f& getPosition() const;
            const Eigen::Vector3f& getVelocity() const;
            float getMass() const;
            float getRadius() const;

            void setPosition(Eigen::Vector3f position);
            void setVelocity(Eigen::Vector3f velocity);





        private:
            Eigen::Vector3f position;
            Eigen::Vector3f velocity;
            float mass;
            float radius;

    };
}

#endif //BODY_H