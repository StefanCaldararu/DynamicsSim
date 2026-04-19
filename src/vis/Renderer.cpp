#include "vis/Renderer.hpp"

namespace Vis {

Renderer::Renderer() : controller(), view() {}

void Renderer::update(const std::vector<Dynamics::Body>& bodies) {
    this->update(bodies, -1.0);
}

void Renderer::update(const std::vector<Dynamics::Body>& bodies, double simTime) {
    currentSimTime = simTime;
    
    controller.setWindowSize(static_cast<float>(view.getWindow().getSize().x), static_cast<float>(view.getWindow().getSize().y));
    
    controller.update(bodies);
    
    view.render(controller.getViewBodies(), controller.getTrails(), controller.getScale(), controller.getScreenCenter(), currentSimTime);
}

bool Renderer::isOpen() {
    return view.isOpen();
}

void Renderer::handleEvent() {
    view.handleEvent();
}

}
