#include "vis/Controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace Vis {

Controller::Controller() 
    : viewBodies(), trails(), trailWorldPositions(), windowWidth(View::WINDOW_X), windowHeight(View::WINDOW_Y), scale(1e9f), screenCenter(windowWidth / 2.0f, windowHeight / 2.0f) {}

void Controller::setWindowSize(float width, float height) {
    windowWidth = width;
    windowHeight = height;
    screenCenter = Eigen::Vector2f(width / 2.0f, height / 2.0f);
}

void Controller::update(const std::vector<Dynamics::Body>& bodies) {
    if (bodies.empty()) {
        viewBodies.clear();
        return;
    }
    
    updateGenericScaling(bodies);
    updateTrails(bodies);
    
    viewBodies.clear();
    for (size_t i = 0; i < bodies.size(); i++) {
        const auto& body = bodies[i];
        Eigen::Vector3d pos = body.getPosition();
        double radius = body.getRadius();
        
        float x = screenCenter.x() + static_cast<float>(pos.x() * scale);
        float y = screenCenter.y() - static_cast<float>(pos.y() * scale);
        
        double draw_radius = std::max(MIN_DRAW_RADIUS, radius * scale * DRAW_RADIUS_FACTOR);
        draw_radius = std::min(draw_radius, MAX_DRAW_RADIUS);
        uint8_t r = View::MAX_COLOR;
        uint8_t g = View::MAX_COLOR;
        uint8_t b = View::MAX_COLOR;
        if (i == 1) {
            r = 0;
            g = 0; 
            b = View::MAX_COLOR;
        } else if (i == 2) {
            r = View::MAX_COLOR;
            g = 0;
            b = 0;
        }
        
        viewBodies.push_back({x, y, static_cast<float>(draw_radius), r, g, b});
    }
}

void Controller::updateGenericScaling(const std::vector<Dynamics::Body>& bodies) {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& body : bodies) {
        Eigen::Vector3d pos = body.getPosition();
        min_x = std::min(min_x, pos.x());
        max_x = std::max(max_x, pos.x());
        min_y = std::min(min_y, pos.y());
        max_y = std::max(max_y, pos.y());
    }

    double max_dist = 0.0;
    max_dist = std::max(max_dist, std::abs(min_x));
    max_dist = std::max(max_dist, std::abs(max_x));
    max_dist = std::max(max_dist, std::abs(min_y));
    max_dist = std::max(max_dist, std::abs(max_y));

    double scale_x = (windowWidth * WINDOW_SCALE_FACTOR) / (max_dist + DIV_0_EPSILON);
    double scale_y = (windowHeight * WINDOW_SCALE_FACTOR) / (max_dist + DIV_0_EPSILON);
    double new_scale = std::min(scale_x, scale_y);

    scale = std::min(static_cast<double>(scale), new_scale);
}

void Controller::updateTrails(const std::vector<Dynamics::Body>& bodies) {
    if (bodies.empty()) {
        trails.clear();
        trailWorldPositions.clear();
        return;
    }
    
    if (trailWorldPositions.size() != bodies.size()) {
        trailWorldPositions.resize(bodies.size());
        trails.resize(bodies.size());
    }
    
    for (size_t i = 0; i < bodies.size(); i++) {
        Eigen::Vector3d pos = bodies[i].getPosition();
        trailWorldPositions[i].push_back(Eigen::Vector2d(pos.x(), pos.y()));
        
        if (trailWorldPositions[i].size() > MAX_TRAIL_LENGTH) {
            trailWorldPositions[i].erase(trailWorldPositions[i].begin());
        }
        
        trails[i].clear();
        for (size_t j = 0; j < trailWorldPositions[i].size(); j++) {
            float x = screenCenter.x() + static_cast<float>(trailWorldPositions[i][j].x() * scale);
            float y = screenCenter.y() - static_cast<float>(trailWorldPositions[i][j].y() * scale);
            uint8_t alpha = static_cast<uint8_t>(View::MAX_COLOR * (static_cast<double>(j) / trailWorldPositions[i].size()));
            
            trails[i].push_back({x, y, alpha});
        }
    }
}

std::vector<ViewBody> Controller::getViewBodies() const {
    return viewBodies;
}

std::vector<std::vector<TrailPoint>> Controller::getTrails() const {
    return trails;
}

float Controller::getScale() const {
    return scale;
}

Eigen::Vector2f Controller::getScreenCenter() const {
    return screenCenter;
}

}
