#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <vector>
#include "dynamics/Body.hpp"

namespace Dynamics {

class Dynamics {
public:

    Dynamics(float G = 1.0f);

    void addBody(const Body& body);

    void update(float dt);

    const std::vector<Body>& getBodies() const;

private:

    std::vector<Body> bodies;

    float G;

    void computeAccelerations(std::vector<Eigen::Vector3f>& accels);
};

}

#endif