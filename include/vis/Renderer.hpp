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

};
}

#endif