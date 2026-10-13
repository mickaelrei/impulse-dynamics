#pragma once

#include <cstddef>
#include <vector>

#include <glm/glm.hpp>

#include "core/body.hpp"
#include "sim/scenarios.hpp"

namespace bench {

constexpr float kReplicaSpacingZ = 36.f;

void setupHeadOnReplica(std::vector<Body>& bodies, float zOffset);
void setupNewtonsCradleReplica(std::vector<Body>& bodies, size_t chainLength, float zOffset);
void setupSimultaneousHitReplica(std::vector<Body>& bodies, float zOffset);

void buildStressLayout(
    std::vector<Body>& bodies,
    Scenario archetype,
    size_t replicas,
    size_t newtonChainLength = 5
);

size_t bodyCountFor(Scenario archetype, size_t replicas, size_t newtonChainLength = 5);

} // namespace bench
