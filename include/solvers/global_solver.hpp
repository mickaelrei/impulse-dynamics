#pragma once

#include "solvers/collision_solver.hpp"

class GlobalImpulseSolver : public CollisionSolver {
public:
    void resolve(
        std::span<Body> bodies,
        const std::vector<std::pair<size_t, size_t>>& pairs,
        float restitution = 1.f
    ) override;
};
