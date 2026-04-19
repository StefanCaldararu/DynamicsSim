#include "vis/view.hpp"
#include <iomanip>
#include <sstream>
#include <iostream>

namespace Vis {

View::View() : infoText(font) {
    window = sf::RenderWindow(sf::VideoMode({800, 600}), "Simulation");
    
    const std::string fontPath = "../Assets/Fonts/Arial.ttf";
    if (font.openFromFile(fontPath)) {
        infoText = sf::Text(font, "", 14);
        infoText.setFillColor(sf::Color::White);
        infoText.setPosition(sf::Vector2f(12., window.getSize().y - 80.));
    }
}

void View::render(const std::vector<ViewBody>& bodies, const std::vector<std::vector<TrailPoint>>& trails, float scale, const Eigen::Vector2f& screenCenter, double simTime) {
    clear();
    renderTrails(trails);
    renderBodies(bodies, scale, screenCenter);
    renderInfoPanel(simTime, bodies.size());
    display();
}

void View::renderBodies(const std::vector<ViewBody>& bodies, float scale, const Eigen::Vector2f& screenCenter) {
    for (const auto& viewBody : bodies) {
        sf::CircleShape circle(viewBody.radius);
        circle.setFillColor(sf::Color(viewBody.r, viewBody.g, viewBody.b));
        circle.setPosition({viewBody.x - viewBody.radius, viewBody.y - viewBody.radius});
        window.draw(circle);
    }
}

void View::renderTrails(const std::vector<std::vector<TrailPoint>>& trails) {
    for (const auto& trail : trails) {
        if (trail.size() > 1) {
            std::vector<sf::Vertex> vertices(trail.size());
            
            for (size_t i = 0; i < trail.size(); i++) {
                vertices[i].position = {trail[i].x, trail[i].y};
                vertices[i].color = sf::Color(255, 255, 255, trail[i].alpha);
            }
            
            window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::LineStrip);
        }
    }
}

void View::renderInfoPanel(double simTime, size_t bodyCount) {
    if (font.getInfo().family.empty()) {
        return;
    }
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Time: " << simTime << "\n";
    oss << "Bodies: " << bodyCount;
    infoText.setString(oss.str());
    infoText.setPosition(sf::Vector2f(12., window.getSize().y - 80.));
    window.draw(infoText);
}

void View::clear() {
    window.clear(sf::Color::Black);
}

void View::display() {
    window.display();
}

bool View::isOpen() const {
    return window.isOpen();
}

void View::handleEvent() {
    while (auto event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window.close();
        }
    }
}

sf::RenderWindow& View::getWindow() {
    return window;
}

}
