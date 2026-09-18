#include "solvers/iterative_solver.hpp"

#include <vector>

#include "math/impulses_util.hpp"

void IterativeSolver::resolve(
    std::span<Body> bodies,
    const std::vector<std::pair<size_t, size_t>>& pairs,
    float restitution
) {
    if (pairs.empty()) return;

    const size_t n = bodies.size();
    std::vector<glm::vec3> positions(n);
    std::vector<glm::vec3> velocities(n);
    std::vector<float> masses(n);

    for (size_t i = 0; i < n; ++i) {
        positions[i] = bodies[i].position;
        velocities[i] = bodies[i].velocity;
        masses[i] = bodies[i].mass;
    }

    const auto dv = resolveSimultaneousCollisions3D(
        positions, velocities, masses, pairs, restitution,
        LinearSolverMethod::GaussSeidel, maxIterations_, tolerance_
    );

    for (size_t i = 0; i < n; ++i) {
        bodies[i].velocity.x += dv(3 * i, 0);
        bodies[i].velocity.y += dv(3 * i + 1, 0);
        bodies[i].velocity.z += dv(3 * i + 2, 0);
    }
}
