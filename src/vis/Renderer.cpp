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

    sf::Vector2f Vis::Renderer::worldToScreen(
        const Eigen::Vector2f& world,
        const Eigen::Vector3f& origin,
        const sf::Vector2f& screen_center,
        float scale)
    {
        float x = screen_center.x + (world.x() - origin.x()) * scale;
        float y = screen_center.y - (world.y() - origin.y()) * scale;
        return sf::Vector2f(x, y);
    }
    


    void Vis::Renderer::update(const std::vector<Dynamics::Body>& bodies){

        if (bodies.size() != 3){
            renderGeneric(bodies);
            return;
        }

        window.clear(sf::Color::Black);
        float width = window.getSize().x;
        float height = window.getSize().y;
        sf::Vector2f screen_center(width / 2.0f, height / 2.0f);

        const auto& p1_body = bodies[1];
        Eigen::Vector3f p1 = p1_body.getPosition();

        Eigen::Vector3f spacecraft = bodies[0].getPosition();
        spacecraftTrailWorld.push_back(Eigen::Vector2f(spacecraft.x(), spacecraft.y()));

        if (spacecraftTrailWorld.size() > maxTrailLength){
            spacecraftTrailWorld.erase(spacecraftTrailWorld.begin());
        }

        float max_dx = 0.0f;
        float max_dy = 0.0f;

        for (const auto& body : bodies) {
            Eigen::Vector3f pos = body.getPosition();
            max_dx = std::max(max_dx, std::abs(pos.x() - p1.x()));
            max_dy = std::max(max_dy, std::abs(pos.y() - p1.y()));
        }

        max_dx *= margin;
        max_dy *= margin;

        float scale_x = (2.0f/3.0f * width) / (max_dx + 1e-5f);
        float scale_y = (0.5f * height) / (max_dy + 1e-5f);

        float new_scale = std::min(scale_x, scale_y);

        if (scale < 0.0f)
            scale = new_scale;
        else if (new_scale < scale)
            scale = new_scale;

        if (spacecraftTrailWorld.size() > 1) {
            std::vector<sf::Vertex> vertices(spacecraftTrailWorld.size());

            for (size_t i = 0; i < spacecraftTrailWorld.size(); i++) {
                sf::Vector2f screen_pos = worldToScreen(spacecraftTrailWorld[i], p1, screen_center, scale);

                float alpha = 255.0f * (float(i) / spacecraftTrailWorld.size());

                vertices[i].position = screen_pos;
                vertices[i].color = sf::Color(255, 255, 255,static_cast<uint8_t>(alpha));
            }

            window.draw(vertices.data(), vertices.size(),sf::PrimitiveType::LineStrip);
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