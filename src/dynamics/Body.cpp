#include "dynamics/Body.hpp"

namespace Dynamics {

Body::Body(Eigen::Vector3d position_, Eigen::Vector3d velocity_, double mass_, double radius_){
    state.resize(6);
    setPosition(position_);
    setVelocity(velocity_);
    mass = mass_;
    radius = radius_;
}

const Eigen::Vector3d Body::getPosition() const{
    return Eigen::Vector3d(state[0], state[1], state[2]);
}

const Eigen::Vector3d Body::getVelocity() const{
    return Eigen::Vector3d(state[3], state[4], state[5]);
}

double Body::getMass() const {
    return mass;
}

double Body::getRadius() const {
    return radius;
}

void Body::setPosition(const Eigen::Vector3d pos) {
    state[0] = pos.x();
    state[1] = pos.y();
    state[2] = pos.z();
}

void Body::setVelocity(const Eigen::Vector3d vel) {
    state[3] = vel.x();
    state[4] = vel.y();
    state[5] = vel.z();
}

std::vector<double> Body::getState() {
    return state;
}

void Body::setState(std::vector<double> state) {
    this->state = state;
}

}