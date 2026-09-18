#pragma once

#include "solvers/collision_solver.hpp"

class IterativeSolver : public CollisionSolver {
public:
    explicit IterativeSolver(size_t maxIterations = 50, float tolerance = 1e-5f)
        : maxIterations_(maxIterations), tolerance_(tolerance) {}

    void resolve(
        std::span<Body> bodies,
        const std::vector<std::pair<size_t, size_t>>& pairs,
        float restitution = 1.f
    ) override;

    void setMaxIterations(size_t maxIterations) { maxIterations_ = maxIterations; }
    size_t maxIterations() const { return maxIterations_; }

    void setTolerance(float tolerance) { tolerance_ = tolerance; }
    float tolerance() const { return tolerance_; }

private:
    size_t maxIterations_ = 50;
    float tolerance_ = 1e-5f;
};
