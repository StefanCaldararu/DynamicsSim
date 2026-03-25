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
        bool isOpen();
        void handleEvent();

    private:
        sf::RenderWindow window;
        void renderGeneric(const std::vector<Dynamics::Body>& bodies);
        float scale = -1.0f;
        float margin = 1.2f;

        std::vector<Eigen::Vector2f> spacecraftTrailWorld;
        size_t maxTrailLength = 8000;

        sf::Vector2f worldToScreen( const Eigen::Vector2f& world, const Eigen::Vector3f& origin, const sf::Vector2f& screen_center, float scale);
};
}

#endif