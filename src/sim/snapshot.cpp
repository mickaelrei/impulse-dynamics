#include "sim/snapshot.hpp"

#include "core/body.hpp"

SimulationSnapshot SimulationSnapshot::capture(const std::vector<Body>& bodies) {
    SimulationSnapshot snap;
    snap.positions.reserve(bodies.size());
    snap.velocities.reserve(bodies.size());
    for (const auto& body : bodies) {
        snap.positions.push_back(body.position);
        snap.velocities.push_back(body.velocity);
    }
    return snap;
}

void SimulationSnapshot::apply(std::vector<Body>& bodies) const {
    for (size_t i = 0; i < bodies.size(); ++i) {
        bodies[i].position = positions[i];
        bodies[i].velocity = velocities[i];
    }
}
