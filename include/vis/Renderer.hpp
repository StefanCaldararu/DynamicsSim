#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <vector>
#include "dynamics/Body.hpp"
namespace Vis {
class Renderer {
    public:
        Renderer();
        void update(const std::vector<Dynamics::Body>& bodies);
        void update(const std::vector<Dynamics::Body>& bodies, double simTime, double jc);
        bool isOpen();
        void handleEvent();

    private:
        void renderGeneric(const std::vector<Dynamics::Body>& bodies);
        sf::Vector2f worldToScreen( const Eigen::Vector2d& world, const Eigen::Vector3d& origin, const sf::Vector2f& screen_center, double scale);
        void renderInfoPanel(double simTime, const std::vector<Dynamics::Body>& bodies, double jc);
        sf::RenderWindow window;
        double scale = -1.0;
        double margin = 1.2;

        std::vector<Eigen::Vector2d> spacecraftTrailWorld;
        size_t maxTrailLength = 8000;

        sf::Font font;
        sf::Text infoText;
};
}

#endif