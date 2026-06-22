#pragma once

#include <span>
#include <utility>
#include <vector>

#include "core/body.hpp"

class CollisionSolver {
public:
    virtual ~CollisionSolver() = default;

    virtual void resolve(
        std::span<Body> bodies,
        const std::vector<std::pair<size_t, size_t>>& pairs,
        float restitution = 1.f
    ) = 0;
};
