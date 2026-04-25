#ifndef VIS_CONTROLLER_HPP
#define VIS_CONTROLLER_HPP

#include <Eigen/Dense>
#include <vector>
#include "dynamics/Body.hpp"
#include "vis/view.hpp"
#include "vis/types.hpp"

namespace Vis {

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

    static constexpr double MIN_DRAW_RADIUS = 3.0;
    static constexpr double MAX_DRAW_RADIUS = 30.0;
    static constexpr double DRAW_RADIUS_FACTOR = 0.02;
    static constexpr double DIV_0_EPSILON = 1e-5;
    static constexpr double WINDOW_SCALE_FACTOR = 0.4;
    static constexpr size_t MAX_TRAIL_LENGTH = 8000;
};

}

#endif