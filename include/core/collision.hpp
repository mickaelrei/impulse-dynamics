#pragma once

#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/body.hpp"

inline std::vector<std::pair<size_t, size_t>> detectSphereCollisions(const std::vector<Body>& bodies) {
    std::vector<std::pair<size_t, size_t>> pairs;
    const size_t n = bodies.size();

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (bodies[i].position == bodies[j].position) continue;
            const float sumRadii = bodies[i].radius + bodies[j].radius;
            if (glm::distance(bodies[i].position, bodies[j].position) <= sumRadii) {
                pairs.emplace_back(i, j);
            }
        }
    }

    return pairs;
}
