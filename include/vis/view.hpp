#ifndef VIS_VIEW_HPP
#define VIS_VIEW_HPP

#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <vector>
#include "Controller.hpp"
#include "vis/types.hpp"
namespace Vis {

class View {
public:
    static constexpr int WINDOW_X = 800;
    static constexpr int WINDOW_Y = 600;
    static constexpr int MAX_COLOR = 255;
    View();
    void render(const std::vector<ViewBody>& bodies, const std::vector<std::vector<TrailPoint>>& trails, float scale, const Eigen::Vector2f& screenCenter, double simTime);
    void clear();
    void display();
    bool isOpen() const;
    void handleEvent();
    sf::RenderWindow& getWindow();
    
private:
    static constexpr int FONT_SIZE = 14;
    static constexpr int INFO_TEXT_X = 12;
    static constexpr int INFO_TEXT_Y = 520;
    void renderBodies(const std::vector<ViewBody>& bodies, float scale, const Eigen::Vector2f& screenCenter);
    void renderTrails(const std::vector<std::vector<TrailPoint>>& trails);
    void renderInfoPanel(double simTime, size_t bodyCount);
    
    sf::RenderWindow window;
    sf::Font font;
    sf::Text infoText;
};

}

#endif