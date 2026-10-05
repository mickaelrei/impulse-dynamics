#pragma once

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "core/body.hpp"
#include "core/penetration_correction.hpp"

struct Plane {
    glm::vec3 normal{0.f, 1.f, 0.f};
    glm::vec3 point{0.f, 0.f, 0.f};
    float restitution = 1.f;
    bool enabled = true;
};

inline Plane makePlane(glm::vec3 normal, glm::vec3 point, float restitution = 1.f) {
    const float len = glm::length(normal);
    if (len > 1e-8f) {
        normal /= len;
    }
    return Plane{
        .normal = normal,
        .point = point,
        .restitution = restitution,
        .enabled = true,
    };
}

struct SpherePlaneContact {
    size_t bodyIndex = 0;
    size_t planeIndex = 0;
    glm::vec3 normal{0.f, 1.f, 0.f};
    float penetration = 0.f;
};

inline float spherePlaneSignedDistance(glm::vec3 center, const Plane& plane) {
    return glm::dot(plane.normal, center - plane.point);
}

inline std::vector<SpherePlaneContact> detectSpherePlaneContacts(
    std::span<const Body> bodies,
    std::span<const Plane> planes
) {
    std::vector<SpherePlaneContact> contacts;

    for (size_t pi = 0; pi < planes.size(); ++pi) {
        const Plane& plane = planes[pi];
        if (!plane.enabled) continue;

        for (size_t bi = 0; bi < bodies.size(); ++bi) {
            const Body& body = bodies[bi];
            if (body.mass <= 0.f) continue;

            const float signedDist = spherePlaneSignedDistance(body.position, plane);
            const float penetration = body.radius - signedDist;
            if (penetration <= 0.f) continue;

            contacts.push_back(SpherePlaneContact{
                bi,
                pi,
                plane.normal,
                penetration,
            });
        }
    }

    return contacts;
}

inline void resolveSpherePlaneImpulses(
    std::span<Body> bodies,
    const std::vector<SpherePlaneContact>& contacts,
    std::span<const Plane> planes
) {
    for (const SpherePlaneContact& contact : contacts) {
        Body& body = bodies[contact.bodyIndex];
        const float e = planes[contact.planeIndex].restitution;
        const glm::vec3& n = contact.normal;

        const float vn = glm::dot(body.velocity, n);
        if (vn >= 0.f && contact.penetration <= 0.f) continue;

        if (vn < 0.f) {
            body.velocity -= (1.f + e) * vn * n;
        }
    }
}

inline void correctSpherePlanePenetration(
    std::span<Body> bodies,
    const std::vector<SpherePlaneContact>& contacts,
    std::span<const Plane> planes,
    PenetrationSettings settings
) {
    for (const SpherePlaneContact& contact : contacts) {
        Body& body = bodies[contact.bodyIndex];
        const Plane& plane = planes[contact.planeIndex];
        const glm::vec3& n = plane.normal;

        const float penetration = body.radius - spherePlaneSignedDistance(body.position, plane);
        const float correctionMag = std::max(0.f, penetration - settings.slop) * settings.percent;
        if (correctionMag <= 0.f) continue;

        body.position += n * correctionMag;
    }
}

inline void resolveSpherePlanePenetration(
    std::span<Body> bodies,
    std::span<const Plane> planes,
    PenetrationSettings settings
) {
    const int iterations = std::max(0, settings.iterations);
    for (int iter = 0; iter < iterations; ++iter) {
        const auto contacts = detectSpherePlaneContacts(
            std::span<const Body>(bodies),
            planes
        );
        if (contacts.empty()) break;
        correctSpherePlanePenetration(bodies, contacts, planes, settings);
    }
}
