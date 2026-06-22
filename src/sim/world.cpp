#include "sim/world.hpp"

#include "core/collision.hpp"

void World::step(float dt, CollisionSolver& solver) {
    for (auto& body : bodies_) {
        body.position += body.velocity * dt;
    }

    const auto pairs = detectSphereCollisions(bodies_);
    if (!pairs.empty()) {
        solver.resolve(bodies_, pairs);
    }
}
