#include <algorithm>
#include <chrono>
#include <string>

#include "tmig/render/render.hpp"
#include "tmig/render/ui.hpp"
#include "tmig/render/window.hpp"
#include "tmig/util/camera_controller.hpp"
#include "tmig/util/time_step.hpp"
#include "tmig/core/input.hpp"

#include "imgui.h"

#include "core/collision.hpp"
#include "core/physics_metrics.hpp"
#include "render/scene_renderer.hpp"
#include "sim/scenarios.hpp"
#include "sim/timeline.hpp"
#include "solvers/collision_solver.hpp"
#include "solvers/global_solver.hpp"
#include "solvers/sequential_solver.hpp"

using namespace tmig;
using Clock = std::chrono::steady_clock;

struct SimMetrics {
    float baselineEnergy = 0.f;
    float baselineMomentum = 0.f;

    void capture(const std::vector<Body>& bodies) {
        baselineEnergy = kineticEnergy(bodies);
        baselineMomentum = glm::length(totalMomentum(bodies));
    }
};

static void resetScenario(
    SimulationTimeline& timeline,
    Scenario scenario,
    int& objectCount,
    SimMetrics& metrics
) {
    const auto& def = getScenarioDef(scenario);
    if (!def.variableBodyCount) {
        objectCount = static_cast<int>(def.defaultBodyCount);
    } else {
        objectCount = std::max(static_cast<int>(def.defaultBodyCount), objectCount);
    }
    timeline.reset(static_cast<size_t>(objectCount), def.setup);
    metrics.capture(timeline.world().bodies());
}

int main() {
    render::init();
    render::ui::init();
    render::setClearColor(glm::vec4{0.f, 0.f, 0.f, 1.f});

    render::Camera camera;
    camera.maxDist = 10000.f;
    camera.setPosition(glm::vec3{0.f, 2.f, 2.f});

    SceneRenderer renderer;
    if (!renderer.init()) {
        return 1;
    }

    GlobalImpulseSolver globalSolver;
    SequentialSolver sequentialSolver;
    CollisionSolver* activeSolver = &globalSolver;

    constexpr float simDt = 1.f / 60.f;
    SimulationTimeline timeline(globalSolver, simDt);
    SimMetrics metrics;

    int scenarioIndex = static_cast<int>(Scenario::NewtonsCradle);
    int objectCount = static_cast<int>(getScenarioDef(Scenario::NewtonsCradle).defaultBodyCount);
    resetScenario(timeline, Scenario::NewtonsCradle, objectCount, metrics);
    renderer.sync(timeline.world().bodies());

    util::TimeStep timeStep;
    util::SmoothFirstPersonCameraController camController;
    camController.moveSpeed = 30.f;

    bool playing = false;
    bool applyBloom = true;
    int solverIndex = 0;
    float stepAccumulator = 0.f;
    float renderTimeMs = 0.f;

    while (!render::window::shouldClose()) {
        core::input::update();
        render::ui::beginFrame();

        if (timeStep.update(render::window::getRuntime())) {
            const std::string title = "impulse-dynamics | FPS: "
                + std::to_string(static_cast<int>(std::round(timeStep.fps())));
            render::window::setTitle(title);
        }

        if (isKeyPressed(core::input::Key::K)) {
            timeline.stepForward();
            renderer.sync(timeline.world().bodies());
        }
        if (isKeyPressed(core::input::Key::J)) {
            timeline.stepBackward();
            renderer.sync(timeline.world().bodies());
        }
        if (isKeyPressed(core::input::Key::ESCAPE)) {
            render::window::setShouldClose(true);
            continue;
        }

        if (playing) {
            stepAccumulator += timeStep.dt();
            while (stepAccumulator >= simDt) {
                timeline.stepForward();
                stepAccumulator -= simDt;
            }
            renderer.sync(timeline.world().bodies());
        }

        const auto& currentScenario = getScenarioDef(static_cast<Scenario>(scenarioIndex));
        const auto& bodies = timeline.world().bodies();
        const float energy = kineticEnergy(bodies);
        const float momentum = glm::length(totalMomentum(bodies));
        const size_t overlaps = detectSphereCollisions(bodies).size();

        const auto viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
        ImGui::SetNextWindowSize(ImVec2(320, 430));

        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        ImGui::Checkbox("Play", &playing);
        ImGui::Checkbox("Bloom", &applyBloom);

        const char* scenarioNames[static_cast<int>(Scenario::Count)];
        for (int i = 0; i < static_cast<int>(Scenario::Count); ++i) {
            scenarioNames[i] = getScenarioDef(static_cast<Scenario>(i)).name;
        }

        if (ImGui::Combo("Scenario", &scenarioIndex, scenarioNames, static_cast<int>(Scenario::Count))) {
            resetScenario(timeline, static_cast<Scenario>(scenarioIndex), objectCount, metrics);
            renderer.sync(timeline.world().bodies());
            stepAccumulator = 0.f;
        }
        ImGui::TextWrapped("%s", currentScenario.description);

        if (currentScenario.variableBodyCount) {
            if (ImGui::InputInt("Objects", &objectCount, 1, 1)) {
                objectCount = std::max(2, objectCount);
            }
        } else {
            ImGui::BeginDisabled();
            ImGui::InputInt("Objects", &objectCount, 1, 1);
            ImGui::EndDisabled();
        }

        if (ImGui::Button("Reset")) {
            resetScenario(timeline, static_cast<Scenario>(scenarioIndex), objectCount, metrics);
            renderer.sync(timeline.world().bodies());
            stepAccumulator = 0.f;
        }

        if (ImGui::Button("Step back")) {
            timeline.stepBackward();
            renderer.sync(timeline.world().bodies());
        }
        ImGui::SameLine();
        if (ImGui::Button("Step forward")) {
            timeline.stepForward();
            renderer.sync(timeline.world().bodies());
        }

        const char* solverNames[] = {"Global", "Sequential"};
        if (ImGui::Combo("Solver", &solverIndex, solverNames, 2)) {
            activeSolver = solverIndex == 0 ? static_cast<CollisionSolver*>(&globalSolver)
                                            : static_cast<CollisionSolver*>(&sequentialSolver);
            timeline.setSolver(*activeSolver);
        }

        ImGui::Separator();
        ImGui::Text("Simulation");
        ImGui::Text("Step %zu / %zu", timeline.currentStep(), timeline.maxStep());
        ImGui::Text("Bodies: %zu", bodies.size());
        ImGui::Text("Overlaps: %zu", overlaps);
        ImGui::Text("Sim dt: %.4f ms", simDt * 1000.f);
        ImGui::Text("Play acc: %.4f ms", stepAccumulator * 1000.f);

        ImGui::Separator();
        ImGui::Text("Physics");
        ImGui::Text("Kinetic energy: %.3f", energy);
        ImGui::Text("Energy drift: %+.3f", energy - metrics.baselineEnergy);
        ImGui::Text("|Momentum|: %.3f", momentum);
        ImGui::Text("Momentum drift: %+.3f", momentum - metrics.baselineMomentum);
        ImGui::Text("Sum |v|: %.3f", totalSpeed(bodies));

        ImGui::Separator();
        ImGui::Text("Performance");
        ImGui::Text("Frame dt: %.3f ms", timeStep.dt() * 1000.f);
        ImGui::Text("FPS: %.1f", timeStep.fps());
        ImGui::Text("Render: %.3f ms", renderTimeMs);

        ImGui::End();

        camController.update(camera, timeStep.dt());

        const auto renderStart = Clock::now();
        renderer.render(camera, SceneRenderOptions{.applyBloom = applyBloom});
        renderTimeMs = std::chrono::duration<float, std::milli>(Clock::now() - renderStart).count();

        render::ui::endFrame();
        render::window::swapBuffers();
    }

    render::ui::terminate();
    return 0;
}
