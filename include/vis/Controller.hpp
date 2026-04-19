#ifndef VIS_CONTROLLER_HPP
#define VIS_CONTROLLER_HPP

#include <Eigen/Dense>
#include <vector>
#include "dynamics/Body.hpp"

namespace Vis {

struct ViewBody {
    float x;
    float y;
    float radius;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct TrailPoint {
    float x;
    float y;
    uint8_t alpha;
};

class Controller {
public:
    Controller();
    
    void update(const std::vector<Dynamics::Body>& bodies);
    std::vector<ViewBody> getViewBodies() const;
    std::vector<std::vector<TrailPoint>> getTrails() const;
    float getScale() const;
    Eigen::Vector2f getScreenCenter() const;
    void setWindowSize(float width, float height);
    
private:
    void updateGenericScaling(const std::vector<Dynamics::Body>& bodies);
    void updateTrails(const std::vector<Dynamics::Body>& bodies);
    
    std::vector<ViewBody> viewBodies;
    std::vector<std::vector<TrailPoint>> trails;
    std::vector<std::vector<Eigen::Vector2d>> trailWorldPositions;
    
    float scale = 1e9f;
    Eigen::Vector2f screenCenter;
    float windowWidth = 800.0f;
    float windowHeight = 600.0f;
    
    static constexpr size_t MAX_TRAIL_LENGTH = 8000;
};

}

#endif