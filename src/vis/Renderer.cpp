#include "vis/Renderer.hpp"
#include <iomanip>
#include <sstream>
#include <iostream>

namespace Vis{
    Renderer::Renderer() : infoText(font){
        this->window = sf::RenderWindow(sf::VideoMode({800, 600}), "Simulation");

        const std::string fontPath = "../Assets/Fonts/Arial.ttf";
        font.openFromFile(fontPath);
        if(!font.getInfo().family.empty()){
            infoText = sf::Text(font, "", 14);
            infoText.setFillColor(sf::Color::White);
            infoText.setPosition(sf::Vector2f(12., 12.));
        }

    }

    void Renderer::renderGeneric(const std::vector<Dynamics::Body>& bodies){
        window.clear(sf::Color::Black);

        for(const auto& body: bodies){
            Eigen::Vector3d pos = body.getPosition();
            double radius =  body.getRadius();

            sf::CircleShape circle(static_cast<float>(radius));
            circle.setFillColor(sf::Color::White);
            circle.setPosition({static_cast<float>(pos.x() - radius), static_cast<float>(pos.y() - radius)});

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
        const Eigen::Vector2d& world,
        const Eigen::Vector3d& origin,
        const sf::Vector2f& screen_center,
        double scale)
    {
        double x = screen_center.x + (world.x() - origin.x()) * scale;
        double y = screen_center.y - (world.y() - origin.y()) * scale;
        return sf::Vector2f(static_cast<float>(x), static_cast<float>(y));
    }

    void Renderer::update(const std::vector<Dynamics::Body>& bodies){
        this->update(bodies, -1., -1.);
    }
    


    void Renderer::update(const std::vector<Dynamics::Body>& bodies, double simTime, double jc){

        if (bodies.size() != 3){
            renderGeneric(bodies);
            return;
        }

        window.clear(sf::Color::Black);
        double width = window.getSize().x;
        double height = window.getSize().y;
        sf::Vector2f screen_center(width / 2.0, height / 2.0);

        const auto& p1_body = bodies[1];
        Eigen::Vector3d p1 = p1_body.getPosition();

        Eigen::Vector3d spacecraft = bodies[0].getPosition();
        spacecraftTrailWorld.push_back(Eigen::Vector2d(spacecraft.x(), spacecraft.y()));

        if (spacecraftTrailWorld.size() > maxTrailLength){
            spacecraftTrailWorld.erase(spacecraftTrailWorld.begin());
        }

        double max_dx = 0.0;
        double max_dy = 0.0;

        for (const auto& body : bodies) {
            Eigen::Vector3d pos = body.getPosition();
            max_dx = std::max(max_dx, std::abs(pos.x() - p1.x()));
            max_dy = std::max(max_dy, std::abs(pos.y() - p1.y()));
        }

        max_dx *= margin;
        max_dy *= margin;

        // Ensure minimum viewport extent to avoid division by near-zero
        double min_extent = 1.1;
        max_dx = std::max(max_dx, min_extent);
        max_dy = std::max(max_dy, min_extent);
        // std::cout << max_dx << " " << max_dy << std::endl;

        double scale_x = (2.0/3.0 * width) / (max_dx + 1e-5);
        double scale_y = (0.5 * height) / (max_dy + 1e-5);

        double new_scale = std::min(scale_x, scale_y);

        if (scale < 0.0)
            scale = new_scale;
        else if (new_scale < scale)
            scale = new_scale;

        if (spacecraftTrailWorld.size() > 1) {
            std::vector<sf::Vertex> vertices(spacecraftTrailWorld.size());

            for (size_t i = 0; i < spacecraftTrailWorld.size(); i++) {
                sf::Vector2f screen_pos = worldToScreen(spacecraftTrailWorld[i], p1, screen_center, scale);

                double alpha = 255.0 * (double(i) / spacecraftTrailWorld.size());

                vertices[i].position = screen_pos;
                vertices[i].color = sf::Color(255, 255, 255,static_cast<uint8_t>(alpha));
            }

            window.draw(vertices.data(), vertices.size(),sf::PrimitiveType::LineStrip);
        }

        for(const auto& body: bodies){
            Eigen::Vector3d pos = body.getPosition();
            double radius = body.getRadius();

            double x = screen_center.x + (pos.x() - p1.x()) * scale;
            double y = screen_center.y - (pos.y() - p1.y()) * scale;

            double draw_radius = std::max(2.0, radius * 0.5);
            sf::CircleShape circle(static_cast<float>(draw_radius));

            if (&body == &bodies[1])
                circle.setFillColor(sf::Color::Blue);
            else if (&body == &bodies[2])
                circle.setFillColor(sf::Color::Red);
            else
                circle.setFillColor(sf::Color::White);

            circle.setPosition({static_cast<float>(x - draw_radius), static_cast<float>(y - draw_radius)});
            window.draw(circle);
        }

        renderInfoPanel(simTime, bodies, jc);

        window.display();
    }

    void Renderer::renderInfoPanel(double simTime, const std::vector<Dynamics::Body>& bodies, double jc){
        if(infoText.getString() == "" && font.getInfo().family.empty()){
            return;
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "Time: " << simTime << "\n";
        oss << "Bodies: " << bodies.size() << "\n";
        oss << "Jacobi Constant: " << jc << "\n";
        infoText.setString(oss.str());
        infoText.setPosition(sf::Vector2f(12., window.getSize().y - 80.));
        window.draw(infoText);
    }

    }