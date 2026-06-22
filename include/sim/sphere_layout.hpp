#pragma once

#include <vector>

#include "core/body.hpp"

inline void defaultSphereLayout(std::vector<Body>& bodies) {
    const size_t n = bodies.size();
    for (size_t i = 0; i < n; ++i) {
        bodies[i].mass = 3.f;
        bodies[i].radius = 3.f;
        bodies[i].position = glm::vec3{0.f};
        bodies[i].velocity = glm::vec3{0.f};
        bodies[i].color = glm::vec4{0.7f, 0.7f, 0.9f, 1.f};

        bodies[i].position.x = 2.f * bodies[i].radius * (static_cast<float>(i) - static_cast<float>(n) * 0.5f);
        if (i == 0) {
            bodies[i].position.x -= 3.f * bodies[i].radius;
            bodies[i].velocity.x = 15.f;
            bodies[i].color = glm::vec4{1.f, 0.3f, 0.3f, 1.f};
        }
    }
}
