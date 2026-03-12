#include "dynamics/Body.hpp"

namespace Dynamics {

Body::Body(const Eigen::Vector3f position_,
           const Eigen::Vector3f velocity_,
           float mass_,
           float radius_)
    : position(position_),
      velocity(velocity_),
      mass(mass_),
      radius(radius_)
{}

const Eigen::Vector3f& Body::getPosition() const {
    return position;
}

const Eigen::Vector3f& Body::getVelocity() const {
    return velocity;
}

float Body::getMass() const {
    return mass;
}

float Body::getRadius() const {
    return radius;
}

void Body::setPosition(const Eigen::Vector3f pos) {
    position = pos;
}

void Body::setVelocity(const Eigen::Vector3f vel) {
    velocity = vel;
}

}