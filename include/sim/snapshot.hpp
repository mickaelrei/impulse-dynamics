#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "core/body.hpp"

struct SimulationSnapshot {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> velocities;

    static SimulationSnapshot capture(const std::vector<Body>& bodies);
    void apply(std::vector<Body>& bodies) const;
};
