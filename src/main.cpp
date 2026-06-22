#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "tmig/render/render.hpp"
#include "tmig/render/ui.hpp"
#include "tmig/render/window.hpp"
#include "tmig/util/camera_controller.hpp"
#include "tmig/util/time_step.hpp"
#include "tmig/core/input.hpp"

#include "imgui.h"

#include "render/scene_renderer.hpp"
#include "sim/sphere_layout.hpp"
#include "sim/timeline.hpp"
#include "solvers/collision_solver.hpp"
#include "solvers/global_solver.hpp"
#include "solvers/sequential_solver.hpp"

using namespace tmig;

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

    int objectCount = 3;
    timeline.reset(static_cast<size_t>(objectCount), defaultSphereLayout);
    renderer.sync(timeline.world().bodies());

    util::TimeStep timeStep;
    util::SmoothFirstPersonCameraController camController;
    camController.moveSpeed = 30.f;

    bool playing = false;
    bool applyBloom = true;
    int solverIndex = 0;
    float stepAccumulator = 0.f;

    while (!render::window::shouldClose()) {
        core::input::update();
        render::ui::beginFrame();

        if (timeStep.update(render::window::getRuntime())) {
            const std::string title = "impulse-dynamics | FPS: "
                + std::to_string(static_cast<int>(std::round(timeStep.fps())));
            render::window::setTitle(title);
        }

        if (isKeyPressed(core::input::Key::SPACE)) {
            playing = !playing;
        }
        if (isKeyPressed(core::input::Key::K)) {
            timeline.stepForward();
            renderer.sync(timeline.world().bodies());
        }
        if (isKeyPressed(core::input::Key::J)) {
            timeline.stepBackward();
            renderer.sync(timeline.world().bodies());
        }

        if (playing) {
            stepAccumulator += timeStep.dt();
            while (stepAccumulator >= simDt) {
                timeline.stepForward();
                stepAccumulator -= simDt;
            }
            renderer.sync(timeline.world().bodies());
        }

        const auto viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
        ImGui::SetNextWindowSize(ImVec2(260, 220));

        ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        ImGui::Checkbox("Play", &playing);
        ImGui::Checkbox("Bloom", &applyBloom);

        if (ImGui::InputInt("Objects", &objectCount, 1, 1)) {
            objectCount = std::max(1, objectCount);
        }

        if (ImGui::Button("Reset")) {
            timeline.reset(static_cast<size_t>(objectCount), defaultSphereLayout);
            renderer.sync(timeline.world().bodies());
            stepAccumulator = 0.f;
        }

        if (ImGui::Button("Step back") || ImGui::IsKeyPressed(ImGuiKey_J)) {
            timeline.stepBackward();
            renderer.sync(timeline.world().bodies());
        }
        ImGui::SameLine();
        if (ImGui::Button("Step forward") || ImGui::IsKeyPressed(ImGuiKey_K)) {
            timeline.stepForward();
            renderer.sync(timeline.world().bodies());
        }

        const char* solverNames[] = {"Global", "Sequential"};
        if (ImGui::Combo("Solver", &solverIndex, solverNames, 2)) {
            activeSolver = solverIndex == 0 ? static_cast<CollisionSolver*>(&globalSolver)
                                            : static_cast<CollisionSolver*>(&sequentialSolver);
            timeline.setSolver(*activeSolver);
        }

        ImGui::Text("Step %zu / %zu", timeline.currentStep(), timeline.maxStep());

        ImGui::End();

        camController.update(camera, timeStep.dt());
        renderer.render(camera, SceneRenderOptions{.applyBloom = applyBloom});

        render::ui::endFrame();
        render::window::swapBuffers();
    }

    render::ui::terminate();
    return 0;
}
