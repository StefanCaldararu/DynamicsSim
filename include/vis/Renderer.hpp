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

};
}

#endif