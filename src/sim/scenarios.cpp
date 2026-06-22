#include "sim/scenarios.hpp"

#include <algorithm>

namespace {

constexpr glm::vec4 kDefaultColor{0.7f, 0.7f, 0.9f, 1.f};
constexpr glm::vec4 kAccentColor{1.f, 0.3f, 0.3f, 1.f};
constexpr glm::vec4 kSecondaryColor{0.3f, 0.9f, 0.5f, 1.f};

Body makeBody(
    glm::vec3 position,
    glm::vec3 velocity,
    float mass,
    float radius,
    glm::vec4 color = kDefaultColor
) {
    return Body{
        .position = position,
        .velocity = velocity,
        .mass = mass,
        .radius = radius,
        .color = color,
    };
}

void placeTouchingChain(std::vector<Body>& bodies, float radius, float mass, float startX) {
    const float spacing = 2.f * radius;
    for (size_t i = 0; i < bodies.size(); ++i) {
        bodies[i] = makeBody(
            glm::vec3{startX + static_cast<float>(i) * spacing, 0.f, 0.f},
            glm::vec3{0.f},
            mass,
            radius
        );
    }
}

float centeredChainStartX(size_t count, float radius) {
    const float spacing = 2.f * radius;
    return -static_cast<float>(count - 1) * spacing * 0.5f;
}

} // namespace

void setupHeadOnCollision(std::vector<Body>& bodies) {
    bodies.resize(2);

    constexpr float radius = 3.f;
    constexpr float mass = 3.f;
    constexpr float gap = 12.f;
    constexpr float speed = 12.f;

    bodies[0] = makeBody(
        glm::vec3{-gap, 0.f, 0.f},
        glm::vec3{speed, 0.f, 0.f},
        mass, radius, kAccentColor
    );
    bodies[1] = makeBody(
        glm::vec3{gap, 0.f, 0.f},
        glm::vec3{-speed, 0.f, 0.f},
        mass, radius, kSecondaryColor
    );
}

void setupNewtonsCradle(std::vector<Body>& bodies) {
    if (bodies.size() < 2) {
        bodies.resize(2);
    }

    constexpr float radius = 3.f;
    constexpr float mass = 3.f;
    constexpr float pullBack = 3.f * radius;
    constexpr float launchSpeed = 15.f;

    placeTouchingChain(bodies, radius, mass, centeredChainStartX(bodies.size(), radius));

    bodies[0].position.x -= pullBack;
    bodies[0].velocity.x = launchSpeed;
    bodies[0].color = kAccentColor;
}

void setupUnequalMass(std::vector<Body>& bodies) {
    bodies.resize(2);

    constexpr float lightRadius = 2.f;
    constexpr float heavyRadius = 4.f;
    constexpr float lightMass = 1.f;
    constexpr float heavyMass = 8.f;

    bodies[0] = makeBody(
        glm::vec3{-14.f, 0.f, 0.f},
        glm::vec3{18.f, 0.f, 0.f},
        lightMass, lightRadius, kAccentColor
    );
    bodies[1] = makeBody(
        glm::vec3{8.f, 0.f, 0.f},
        glm::vec3{0.f},
        heavyMass, heavyRadius, kDefaultColor
    );
}

void setupSimultaneousHit(std::vector<Body>& bodies) {
    bodies.resize(3);

    constexpr float radius = 3.f;
    constexpr float mass = 3.f;
    constexpr float gap = 12.f;
    constexpr float speed = 12.f;

    bodies[0] = makeBody(
        glm::vec3{-gap, 0.f, 0.f},
        glm::vec3{speed, 0.f, 0.f},
        mass, radius, kAccentColor
    );
    bodies[1] = makeBody(
        glm::vec3{0.f, 0.f, 0.f},
        glm::vec3{0.f},
        mass, radius, kDefaultColor
    );
    bodies[2] = makeBody(
        glm::vec3{gap, 0.f, 0.f},
        glm::vec3{-speed, 0.f, 0.f},
        mass, radius, kSecondaryColor
    );
}

void setupDoubleEndedHit(std::vector<Body>& bodies) {
    bodies.resize(4);

    constexpr float radius = 3.f;
    constexpr float mass = 3.f;
    constexpr float speed = 12.f;

    placeTouchingChain(bodies, radius, mass, centeredChainStartX(4, radius));

    bodies[0].velocity.x = speed;
    bodies[0].color = kAccentColor;
    bodies[3].velocity.x = -speed;
    bodies[3].color = kSecondaryColor;
}

const ScenarioDef& getScenarioDef(Scenario scenario) {
    static const ScenarioDef defs[] = {
        {
            "Head-on collision",
            "Two equal balls approach on the X axis and collide.",
            2, false, setupHeadOnCollision,
        },
        {
            "Newton's cradle",
            "N balls in a row; the first is pulled back and launched into the chain.",
            5, true, setupNewtonsCradle,
        },
        {
            "Unequal mass",
            "A light ball hits a heavier ball at rest.",
            2, false, setupUnequalMass,
        },
        {
            "Simultaneous hit",
            "Two balls collide with a stationary center ball at the same time.",
            3, false, setupSimultaneousHit,
        },
        {
            "Double-ended hit",
            "Four touching balls; outer balls move inward simultaneously.",
            4, false, setupDoubleEndedHit,
        },
    };

    const int index = static_cast<int>(scenario);
    return defs[std::clamp(index, 0, static_cast<int>(Scenario::Count) - 1)];
}

const char* scenarioName(Scenario scenario) {
    return getScenarioDef(scenario).name;
}
