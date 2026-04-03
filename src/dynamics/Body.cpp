#include "dynamics/Body.hpp"

namespace Dynamics {

Body::Body(const Eigen::Vector3d position_,
           const Eigen::Vector3d velocity_,
           double mass_,
           double radius_)
    : position(position_),
      velocity(velocity_),
      mass(mass_),
      radius(radius_)
{}

const Eigen::Vector3d& Body::getPosition() const {
    return position;
}

const Eigen::Vector3d& Body::getVelocity() const {
    return velocity;
}

double Body::getMass() const {
    return mass;
}

double Body::getRadius() const {
    return radius;
}

void Body::setPosition(const Eigen::Vector3d pos) {
    position = pos;
}

void Body::setVelocity(const Eigen::Vector3d vel) {
    velocity = vel;
}

}