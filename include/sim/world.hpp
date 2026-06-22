#pragma once

#include <vector>

#include "core/body.hpp"
#include "solvers/collision_solver.hpp"

class World {
public:
    const std::vector<Body>& bodies() const { return bodies_; }
    std::vector<Body>& bodies() { return bodies_; }

    void resize(size_t count) { bodies_.resize(count); }

    void step(float dt, CollisionSolver& solver);

private:
    std::vector<Body> bodies_;
};
