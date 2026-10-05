#pragma once

#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/body.hpp"
#include "core/penetration_correction.hpp"
#include "core/plane.hpp"
#include "solvers/collision_solver.hpp"

class World {
public:
    const std::vector<Body>& bodies() const { return bodies_; }
    std::vector<Body>& bodies() { return bodies_; }

    void resize(size_t count) { bodies_.resize(count); }

    void step(float dt, CollisionSolver& solver);

    bool penetrationCorrectionEnabled() const { return penetrationCorrectionEnabled_; }
    void setPenetrationCorrectionEnabled(bool enabled) { penetrationCorrectionEnabled_ = enabled; }

    const PenetrationSettings& penetrationSettings() const { return penetrationSettings_; }
    void setPenetrationSettings(PenetrationSettings settings) { penetrationSettings_ = settings; }

    bool gravityEnabled() const { return gravityEnabled_; }
    void setGravityEnabled(bool enabled) { gravityEnabled_ = enabled; }

    glm::vec3 gravity() const { return gravity_; }
    void setGravity(glm::vec3 gravity) { gravity_ = gravity; }

    const std::vector<Plane>& planes() const { return planes_; }
    std::vector<Plane>& planes() { return planes_; }

    void clearPlanes() { planes_.clear(); }
    void addPlane(Plane plane) { planes_.push_back(std::move(plane)); }
    void setPlanes(std::vector<Plane> planes) { planes_ = std::move(planes); }

private:
    std::vector<Body> bodies_;
    std::vector<Plane> planes_;
    glm::vec3 gravity_{0.f, -9.81f, 0.f};
    bool gravityEnabled_ = false;
    bool penetrationCorrectionEnabled_ = true;
    PenetrationSettings penetrationSettings_{};
};
