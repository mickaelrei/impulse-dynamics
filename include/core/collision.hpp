#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/body.hpp"

struct SphereContact {
    size_t i = 0;
    size_t j = 0;
    glm::vec3 normal{0.f, 1.f, 0.f};
    float penetration = 0.f;
};

inline std::vector<SphereContact> detectSphereContacts(std::span<const Body> bodies) {
    std::vector<SphereContact> contacts;
    const size_t n = bodies.size();

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (bodies[i].position == bodies[j].position) continue;

            const glm::vec3 delta = bodies[j].position - bodies[i].position;
            const float dist = glm::length(delta);
            if (dist <= 1e-8f) continue;

            const float sumRadii = bodies[i].radius + bodies[j].radius;
            const float penetration = sumRadii - dist;
            if (penetration <= 0.f) continue;

            contacts.push_back(SphereContact{
                i,
                j,
                delta / dist,
                penetration,
            });
        }
    }

    return contacts;
}

inline std::vector<std::pair<size_t, size_t>> detectSphereCollisions(const std::vector<Body>& bodies) {
    const auto contacts = detectSphereContacts(std::span<const Body>(bodies));
    std::vector<std::pair<size_t, size_t>> pairs;
    pairs.reserve(contacts.size());
    for (const SphereContact& contact : contacts) {
        pairs.emplace_back(contact.i, contact.j);
    }
    return pairs;
}
