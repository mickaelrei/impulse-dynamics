#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "core/body.hpp"

inline float kineticEnergy(const std::vector<Body>& bodies) {
    float energy = 0.f;
    for (const auto& body : bodies) {
        const float speedSq = glm::dot(body.velocity, body.velocity);
        energy += 0.5f * body.mass * speedSq;
    }
    return energy;
}

inline glm::vec3 totalMomentum(const std::vector<Body>& bodies) {
    glm::vec3 momentum{0.f};
    for (const auto& body : bodies) {
        momentum += body.mass * body.velocity;
    }
    return momentum;
}

inline float totalSpeed(const std::vector<Body>& bodies) {
    float speed = 0.f;
    for (const auto& body : bodies) {
        speed += glm::length(body.velocity);
    }
    return speed;
}
