#include "bench/bench_layouts.hpp"

#include <algorithm>

namespace bench {

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

void placeTouchingChainAt(
    std::vector<Body>& bodies,
    size_t offset,
    size_t count,
    float radius,
    float mass,
    float startX,
    float zOffset
) {
    const float spacing = 2.f * radius;
    bodies.resize(offset + count);
    for (size_t i = 0; i < count; ++i) {
        bodies[offset + i] = makeBody(
            glm::vec3{startX + static_cast<float>(i) * spacing, 0.f, zOffset},
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

void setupHeadOnReplica(std::vector<Body>& bodies, float zOffset) {
    constexpr float radius = 3.f;
    constexpr float mass = 3.f;
    constexpr float speed = 12.f;
    // Already touching along X so the first timed step always resolves contact(s).
    const float centerSep = radius;

    const size_t offset = bodies.size();
    bodies.resize(offset + 2);

    bodies[offset] = makeBody(
        glm::vec3{-centerSep, 0.f, zOffset},
        glm::vec3{speed, 0.f, 0.f},
        mass,
        radius,
        kAccentColor
    );
    bodies[offset + 1] = makeBody(
        glm::vec3{centerSep, 0.f, zOffset},
        glm::vec3{-speed, 0.f, 0.f},
        mass,
        radius,
        kSecondaryColor
    );
}

void setupNewtonsCradleReplica(std::vector<Body>& bodies, size_t chainLength, float zOffset) {
    if (chainLength < 2) {
        chainLength = 2;
    }

    constexpr float radius = 3.f;
    constexpr float mass = 3.f;
    constexpr float pullBack = 3.f * radius;
    constexpr float launchSpeed = 15.f;

    const size_t offset = bodies.size();
    placeTouchingChainAt(
        bodies,
        offset,
        chainLength,
        radius,
        mass,
        centeredChainStartX(chainLength, radius),
        zOffset
    );

    bodies[offset].position.x -= pullBack;
    bodies[offset].velocity.x = launchSpeed;
    bodies[offset].color = kAccentColor;
}

void setupSimultaneousHitReplica(std::vector<Body>& bodies, float zOffset) {
    constexpr float radius = 3.f;
    constexpr float mass = 3.f;
    constexpr float speed = 12.f;
    const float spacing = 2.f * radius;

    const size_t offset = bodies.size();
    bodies.resize(offset + 3);

    bodies[offset] = makeBody(
        glm::vec3{-spacing, 0.f, zOffset},
        glm::vec3{speed, 0.f, 0.f},
        mass,
        radius,
        kAccentColor
    );
    bodies[offset + 1] = makeBody(
        glm::vec3{0.f, 0.f, zOffset},
        glm::vec3{0.f},
        mass,
        radius,
        kDefaultColor
    );
    bodies[offset + 2] = makeBody(
        glm::vec3{spacing, 0.f, zOffset},
        glm::vec3{-speed, 0.f, 0.f},
        mass,
        radius,
        kSecondaryColor
    );
}

size_t bodyCountFor(Scenario archetype, size_t replicas, size_t newtonChainLength) {
    const ScenarioDef& def = getScenarioDef(archetype);
    size_t perReplica = def.defaultBodyCount;
    if (archetype == Scenario::NewtonsCradle) {
        perReplica = std::max(newtonChainLength, size_t{2});
    }
    return perReplica * replicas;
}

void buildStressLayout(
    std::vector<Body>& bodies,
    Scenario archetype,
    size_t replicas,
    size_t newtonChainLength
) {
    bodies.clear();
    bodies.reserve(bodyCountFor(archetype, replicas, newtonChainLength));

    for (size_t r = 0; r < replicas; ++r) {
        const float z = static_cast<float>(r) * kReplicaSpacingZ;
        switch (archetype) {
        case Scenario::HeadOnCollision:
            setupHeadOnReplica(bodies, z);
            break;
        case Scenario::NewtonsCradle:
            setupNewtonsCradleReplica(bodies, newtonChainLength, z);
            break;
        case Scenario::SimultaneousHit:
            setupSimultaneousHitReplica(bodies, z);
            break;
        default:
            break;
        }
    }
}

} // namespace bench
