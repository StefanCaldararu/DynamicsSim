#include "vis/Renderer.hpp"

namespace Vis{
    Renderer::Renderer(){
        this->window = sf::RenderWindow(sf::VideoMode({800, 600}), "Simulation");
    }

    void Renderer::update(const std::vector<Dynamics::Body>& bodies){
        window.clear(sf::Color::Black);

        for(const auto& body: bodies){
            Eigen::Vector3f pos = body.getPosition();
            float radius =  body.getRadius();

            sf::CircleShape circle(radius);
            circle.setFillColor(sf::Color::White);
            circle.setPosition({pos.x() - radius, pos.y() - radius});

            window.draw(circle);
        }
        window.display();
    }

    bool Renderer::isOpen(){
        return window.isOpen();
    }

    void Renderer::handleEvent(){
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
        }
    }
}