#pragma once

#include "solvers/collision_solver.hpp"

// Resolves collisions pair-by-pair in order. Does not handle simultaneous
// contacts globally — useful as a contrast to GlobalImpulseSolver.
class SequentialSolver : public CollisionSolver {
public:
    void resolve(
        std::span<Body> bodies,
        const std::vector<std::pair<size_t, size_t>>& pairs,
        float restitution = 1.f
    ) override;

private:
    static void solve1D(
        glm::vec3 v1, float m1,
        glm::vec3 v2, float m2,
        float restitution,
        glm::vec3* out1, glm::vec3* out2
    );

    static void solvePair(
        glm::vec3 p1, glm::vec3 v1, float m1,
        glm::vec3 p2, glm::vec3 v2, float m2,
        float restitution,
        glm::vec3* out1, glm::vec3* out2
    );
};
