#pragma once

#include <glm/glm.hpp>

struct Body {
    glm::vec3 position{0.f};
    glm::vec3 velocity{0.f};
    float mass = 1.f;
    float radius = 1.f;
    glm::vec4 color{0.7f, 0.7f, 0.9f, 1.f};
};
