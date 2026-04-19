#ifndef RENDERER_H
#define RENDERER_H

#include <vector>
#include "dynamics/Body.hpp"
#include "vis/Controller.hpp"
#include "vis/view.hpp"

namespace Vis {

class Renderer {
    public:
        Renderer();
        void update(const std::vector<Dynamics::Body>& bodies);
        void update(const std::vector<Dynamics::Body>& bodies, double simTime);
        bool isOpen();
        void handleEvent();

    private:
        Controller controller;
        View view;
        double currentSimTime = -1.0;
};

}

#endif