#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "core/body.hpp"

enum class Scenario : int {
    HeadOnCollision = 0,
    NewtonsCradle,
    UnequalMass,
    SimultaneousHit,
    DoubleEndedHit,
    Count
};

struct ScenarioDef {
    const char* name;
    const char* description;
    size_t defaultBodyCount;
    bool variableBodyCount;
    std::function<void(std::vector<Body>&)> setup;
};

const ScenarioDef& getScenarioDef(Scenario scenario);
const char* scenarioName(Scenario scenario);

void setupHeadOnCollision(std::vector<Body>& bodies);
void setupNewtonsCradle(std::vector<Body>& bodies);
void setupUnequalMass(std::vector<Body>& bodies);
void setupSimultaneousHit(std::vector<Body>& bodies);
void setupDoubleEndedHit(std::vector<Body>& bodies);
