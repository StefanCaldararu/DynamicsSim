#ifndef TYPES_HPP
#define TYPES_HPP

namespace Vis {

struct ViewBody {
    float x;
    float y;
    float radius;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct TrailPoint {
    float x;
    float y;
    uint8_t alpha;
};
}

#endif