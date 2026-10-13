#include "sim/timeline.hpp"

#include <chrono>

#include "sim/snapshot.hpp"

using Clock = std::chrono::steady_clock;

SimulationTimeline::SimulationTimeline(CollisionSolver& solver, float dt)
    : solver_(&solver), dt_(dt) {}

void SimulationTimeline::reset(size_t count, const std::function<void(std::vector<Body>&)>& initFn) {
    world_.resize(count);
    initFn(world_.bodies());

    history_.clear();
    history_.push_back(SimulationSnapshot::capture(world_.bodies()));
    cursor_ = 0;
    lastPhysicsStepMs_ = 0.f;
    averagePhysicsStepMs_ = 0.f;
    physicsStepSamples_ = 0;
}

void SimulationTimeline::restore(size_t index) {
    history_[index].apply(world_.bodies());
}

void SimulationTimeline::stepForward() {
    if (cursor_ < history_.size() - 1) {
        ++cursor_;
        restore(cursor_);
        return;
    }

    const auto stepStart = Clock::now();
    world_.step(dt_, *solver_);
    lastPhysicsStepMs_ = std::chrono::duration<float, std::milli>(Clock::now() - stepStart).count();
    ++physicsStepSamples_;
    averagePhysicsStepMs_ += (lastPhysicsStepMs_ - averagePhysicsStepMs_) / static_cast<float>(physicsStepSamples_);

    history_.push_back(SimulationSnapshot::capture(world_.bodies()));
    cursor_ = history_.size() - 1;
}

void SimulationTimeline::stepBackward() {
    if (cursor_ == 0) return;
    --cursor_;
    restore(cursor_);
}

void SimulationTimeline::jumpTo(size_t step) {
    if (step >= history_.size()) return;
    cursor_ = step;
    restore(cursor_);
}
