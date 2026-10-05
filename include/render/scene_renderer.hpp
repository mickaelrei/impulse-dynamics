#pragma once

#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "tmig/render/camera.hpp"
#include "tmig/render/framebuffer.hpp"
#include "tmig/render/instanced_mesh.hpp"
#include "tmig/render/postprocessing/bloom.hpp"
#include "tmig/render/shader.hpp"
#include "tmig/render/texture2D.hpp"
#include "tmig/render/uniform_buffer.hpp"
#include "tmig/util/shapes.hpp"

#include "core/body.hpp"
#include "core/plane.hpp"

struct SceneRenderOptions {
    bool applyBloom = true;
};

class SceneRenderer {
public:
    SceneRenderer();
    ~SceneRenderer();

    SceneRenderer(const SceneRenderer&) = delete;
    SceneRenderer& operator=(const SceneRenderer&) = delete;

    bool init();
    void sync(const std::vector<Body>& bodies, std::span<const Plane> planes = {});
    void render(const tmig::render::Camera& camera, const SceneRenderOptions& options);

    struct InstanceData {
        glm::vec4 color;
        glm::mat4 model;
    };

private:
    struct SceneData {
        glm::mat4 projection;
        glm::mat4 view;
        glm::vec3 viewPos;
    };

    static glm::mat4 planeInstanceMatrix(const Plane& plane, float halfExtent);

    tmig::render::ShaderProgram meshShader_;
    tmig::render::InstancedMesh<tmig::util::GeneralVertex, InstanceData> mesh_;
    tmig::render::InstancedMesh<tmig::util::GeneralVertex, InstanceData> planeMesh_;

    tmig::render::DataBuffer<tmig::util::GeneralVertex>* vertexBuffer_ = nullptr;
    tmig::render::DataBuffer<InstanceData>* instanceBuffer_ = nullptr;
    tmig::render::DataBuffer<uint32_t>* indexBuffer_ = nullptr;

    tmig::render::DataBuffer<tmig::util::GeneralVertex>* planeVertexBuffer_ = nullptr;
    tmig::render::DataBuffer<InstanceData>* planeInstanceBuffer_ = nullptr;
    tmig::render::DataBuffer<uint32_t>* planeIndexBuffer_ = nullptr;

    tmig::render::UniformBuffer<SceneData> ubo_;
    tmig::render::Framebuffer sceneFramebuffer_;
    tmig::render::Texture2D sceneDepthTexture_;
    tmig::render::Texture2D sceneOutputTexture_;
    tmig::render::postprocessing::BloomEffect bloomEffect_;

    std::vector<InstanceData> instances_;
    std::vector<InstanceData> planeInstances_;

    static constexpr float kPlaneVisualHalfExtent = 120.f;
    static constexpr glm::vec4 kPlaneColor{0.52f, 0.55f, 0.58f, 1.f};
};
