#include "vis/Renderer.hpp"

namespace Vis{
    Renderer::Renderer(){
        this->window = sf::RenderWindow(sf::VideoMode({800, 600}), "Simulation");
    }

    void Renderer::renderGeneric(const std::vector<Dynamics::Body>& bodies){
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

    void Renderer::update(const std::vector<Dynamics::Body>& bodies){

        if (bodies.size() != 3){
            renderGeneric(bodies);
            return;
        }

        window.clear(sf::Color::Black);
        float width = window.getSize().x;
        float height = window.getSize().y;

        sf::Vector2f screen_center(width / 2.0f, height / 2.0f);

        const auto& p1_body = bodies[1];
        const auto& p2_body = bodies[2];

        Eigen::Vector3f p1 = p1_body.getPosition();

        float max_dx = 0.0f;
        float max_dy = 0.0f;

        for (const auto& body : bodies) {
            Eigen::Vector3f pos = body.getPosition();

            max_dx = std::max(max_dx, std::abs(pos.x() - p1.x()));
            max_dy = std::max(max_dy, std::abs(pos.y() - p1.y()));
        }

        max_dx *= 1.2f;
        max_dy *= 1.2f;

        float scale_x = (2.0f/3.0f * width) / (max_dx + 1e-5f);
        float scale_y = (0.5f * height) / (max_dy + 1e-5f);

        float new_scale = std::min(scale_x, scale_y);

        if (scale < 0.0f) {
            scale = new_scale; 
        } else if (new_scale < scale) {
            scale = new_scale; 
        }

        for(const auto& body: bodies){
            Eigen::Vector3f pos = body.getPosition();
            float radius = body.getRadius();

            float x = screen_center.x + (pos.x() - p1.x()) * scale;
            float y = screen_center.y - (pos.y() - p1.y()) * scale;

            float draw_radius = std::max(2.0f, radius * 0.5f);

            sf::CircleShape circle(draw_radius);

            if (&body == &bodies[1])
                circle.setFillColor(sf::Color::Blue);
            else if (&body == &bodies[2])
                circle.setFillColor(sf::Color::Red);
            else
                circle.setFillColor(sf::Color::White);

            circle.setPosition({x - draw_radius, y - draw_radius});

            window.draw(circle);
        }

        window.display();
    }
}