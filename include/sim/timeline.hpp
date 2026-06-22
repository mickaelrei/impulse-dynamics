#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "sim/snapshot.hpp"
#include "sim/world.hpp"
#include "solvers/collision_solver.hpp"

class SimulationTimeline {
public:
    SimulationTimeline(CollisionSolver& solver, float dt);

    World& world() { return world_; }
    const World& world() const { return world_; }

    void setSolver(CollisionSolver& solver) { solver_ = &solver; }

    void reset(size_t count, const std::function<void(std::vector<Body>&)>& initFn);

    void stepForward();
    void stepBackward();
    void jumpTo(size_t step);

    size_t currentStep() const { return cursor_; }
    size_t maxStep() const { return history_.empty() ? 0 : history_.size() - 1; }

    float dt() const { return dt_; }

private:
    void restore(size_t index);

    World world_;
    CollisionSolver* solver_;
    float dt_;
    std::vector<SimulationSnapshot> history_;
    size_t cursor_ = 0;
};
