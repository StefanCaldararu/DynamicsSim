#include "vis/Controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace Vis {

Controller::Controller() 
    : viewBodies(), trails(), trailWorldPositions(), windowWidth(800.0f), windowHeight(600.0f), scale(1e9f), screenCenter(windowWidth / 2.0f, windowHeight / 2.0f) {}

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
        
        double draw_radius = std::max(3.0, radius * scale * 0.02);
        draw_radius = std::min(draw_radius, 30.0);
        //TODO: fix coloring if needed, magic numbers
        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
        if (i == 1) {
            r = 0;
            g = 0; 
            b = 255;
        } else if (i == 2) {
            r = 255;
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

    double scale_x = (windowWidth / 2.0 * 0.8) / (max_dist + 1e-5);
    double scale_y = (windowHeight / 2.0 * 0.8) / (max_dist + 1e-5);
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
            uint8_t alpha = static_cast<uint8_t>(255.0 * (static_cast<double>(j) / trailWorldPositions[i].size()));
            
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
