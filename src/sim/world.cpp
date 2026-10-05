#include "sim/world.hpp"

#include <span>

#include "core/collision.hpp"
#include "core/penetration_correction.hpp"
#include "core/plane.hpp"

void World::step(float dt, CollisionSolver& solver) {
    if (gravityEnabled_) {
        for (auto& body : bodies_) {
            if (body.mass > 0.f) {
                body.velocity += gravity_ * dt;
            }
        }
    }

    for (auto& body : bodies_) {
        body.position += body.velocity * dt;
    }

    const auto contacts = detectSphereContacts(std::span<const Body>(bodies_));
    if (!contacts.empty()) {
        std::vector<std::pair<size_t, size_t>> pairs;
        pairs.reserve(contacts.size());
        for (const SphereContact& contact : contacts) {
            pairs.emplace_back(contact.i, contact.j);
        }
        solver.resolve(bodies_, pairs);
    }

    if (penetrationCorrectionEnabled_) {
        resolveSpherePenetration(std::span(bodies_), penetrationSettings_);
    }

    if (!planes_.empty()) {
        const auto planeContacts = detectSpherePlaneContacts(
            std::span<const Body>(bodies_),
            std::span<const Plane>(planes_)
        );
        if (!planeContacts.empty()) {
            resolveSpherePlaneImpulses(
                std::span(bodies_),
                planeContacts,
                std::span<const Plane>(planes_)
            );
        }

        if (penetrationCorrectionEnabled_) {
            resolveSpherePlanePenetration(
                std::span(bodies_),
                std::span<const Plane>(planes_),
                penetrationSettings_
            );
        }
    }
}
