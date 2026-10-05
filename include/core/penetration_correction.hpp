#pragma once

#include <algorithm>
#include <cmath>
#include <span>

#include "core/body.hpp"
#include "core/collision.hpp"

struct PenetrationSettings {
    float percent = 0.8f;
    float slop = 1e-3f;
    int iterations = 3;
};

inline void correctSpherePenetration(
    std::span<Body> bodies,
    const std::vector<SphereContact>& contacts,
    PenetrationSettings settings
) {
    for (const SphereContact& contact : contacts) {
        Body& a = bodies[contact.i];
        Body& b = bodies[contact.j];

        const float invMassA = (a.mass > 0.f) ? (1.f / a.mass) : 0.f;
        const float invMassB = (b.mass > 0.f) ? (1.f / b.mass) : 0.f;
        const float invSum = invMassA + invMassB;
        if (invSum <= 0.f) continue;

        const glm::vec3 delta = b.position - a.position;
        const float dist = glm::length(delta);
        if (dist <= 1e-8f) continue;

        const glm::vec3 normal = delta / dist;
        const float sumRadii = a.radius + b.radius;
        const float penetration = sumRadii - dist;
        const float correctionMag = std::max(0.f, penetration - settings.slop) * settings.percent;
        if (correctionMag <= 0.f) continue;

        const glm::vec3 correction = normal * correctionMag;
        a.position -= correction * (invMassA / invSum);
        b.position += correction * (invMassB / invSum);
    }
}

inline void resolveSpherePenetration(std::span<Body> bodies, PenetrationSettings settings) {
    const int iterations = std::max(0, settings.iterations);
    for (int iter = 0; iter < iterations; ++iter) {
        const auto contacts = detectSphereContacts(std::span<const Body>(bodies));
        if (contacts.empty()) break;
        correctSpherePenetration(bodies, contacts, settings);
    }
}
