#include "render/scene_renderer.hpp"

#include <cmath>
#include <iostream>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>

#include "tmig/render/framebuffer.hpp"
#include "tmig/render/window.hpp"
#include "tmig/util/postprocessing.hpp"
#include "tmig/util/resources.hpp"

using namespace tmig;

namespace {

void configureInstancedMesh(
    render::InstancedMesh<util::GeneralVertex, SceneRenderer::InstanceData>& mesh,
    render::DataBuffer<util::GeneralVertex>* vertexBuffer,
    render::DataBuffer<SceneRenderer::InstanceData>* instanceBuffer,
    render::DataBuffer<uint32_t>* indexBuffer
) {
    mesh.setAttributes({
        render::VertexAttributeType::FLOAT3,
        render::VertexAttributeType::FLOAT3,
        render::VertexAttributeType::FLOAT2,
    }, {
        render::VertexAttributeType::FLOAT4,
        render::VertexAttributeType::MAT4x4,
    });
    mesh.setInstanceBuffer(instanceBuffer);
    mesh.setIndexBuffer(indexBuffer);
    mesh.setVertexBuffer(vertexBuffer);
}

void buildUnitQuadMesh(
    std::vector<util::GeneralVertex>& vertices,
    std::vector<uint32_t>& indices
) {
    const glm::vec3 normal{0.f, 0.f, 1.f};
    vertices = {
        util::GeneralVertex{.position = {-0.5f, -0.5f, 0.f}, .normal = normal, .uv = {0.f, 0.f}},
        util::GeneralVertex{.position = {0.5f, -0.5f, 0.f}, .normal = normal, .uv = {1.f, 0.f}},
        util::GeneralVertex{.position = {0.5f, 0.5f, 0.f}, .normal = normal, .uv = {1.f, 1.f}},
        util::GeneralVertex{.position = {-0.5f, 0.5f, 0.f}, .normal = normal, .uv = {0.f, 1.f}},
    };
    indices = {0, 1, 2, 2, 3, 0};
}

} // namespace

SceneRenderer::SceneRenderer() {
    ubo_.bindTo(0);
}

SceneRenderer::~SceneRenderer() {
    delete vertexBuffer_;
    delete instanceBuffer_;
    delete indexBuffer_;
    delete planeVertexBuffer_;
    delete planeInstanceBuffer_;
    delete planeIndexBuffer_;
}

glm::mat4 SceneRenderer::planeInstanceMatrix(const Plane& plane, float halfExtent) {
    glm::vec3 n = glm::normalize(plane.normal);
    const glm::vec3 reference = std::abs(n.y) < 0.99f ? glm::vec3{0.f, 1.f, 0.f} : glm::vec3{1.f, 0.f, 0.f};
    const glm::vec3 tangent = glm::normalize(glm::cross(reference, n));
    const glm::vec3 bitangent = glm::cross(n, tangent);

    glm::mat4 model{1.f};
    model[0] = glm::vec4(tangent * halfExtent, 0.f);
    model[1] = glm::vec4(bitangent * halfExtent, 0.f);
    model[2] = glm::vec4(n, 0.f);
    model[3] = glm::vec4(plane.point, 1.f);
    return model;
}

bool SceneRenderer::init() {
    if (!meshShader_.compileFromFiles(
        util::getResourcePath("shaders/instanced.vert"),
        util::getResourcePath("shaders/instanced_bloom.frag")
    )) {
        std::cerr << "Failed loading instanced shader\n";
        return false;
    }

    std::vector<util::GeneralVertex> sphereVertices;
    std::vector<uint32_t> sphereIndices;
    util::generateSphereMesh([&](auto v) { sphereVertices.push_back(v); }, sphereIndices, 30);

    vertexBuffer_ = new render::DataBuffer<util::GeneralVertex>;
    instanceBuffer_ = new render::DataBuffer<InstanceData>;
    indexBuffer_ = new render::DataBuffer<uint32_t>;
    vertexBuffer_->setData(sphereVertices);
    indexBuffer_->setData(sphereIndices);
    configureInstancedMesh(mesh_, vertexBuffer_, instanceBuffer_, indexBuffer_);

    std::vector<util::GeneralVertex> quadVertices;
    std::vector<uint32_t> quadIndices;
    buildUnitQuadMesh(quadVertices, quadIndices);

    planeVertexBuffer_ = new render::DataBuffer<util::GeneralVertex>;
    planeInstanceBuffer_ = new render::DataBuffer<InstanceData>;
    planeIndexBuffer_ = new render::DataBuffer<uint32_t>;
    planeVertexBuffer_->setData(quadVertices);
    planeIndexBuffer_->setData(quadIndices);
    configureInstancedMesh(planeMesh_, planeVertexBuffer_, planeInstanceBuffer_, planeIndexBuffer_);

    sceneOutputTexture_.setWrapS(render::TextureWrapMode::CLAMP_TO_EDGE);
    sceneOutputTexture_.setWrapT(render::TextureWrapMode::CLAMP_TO_EDGE);

    const auto status = sceneFramebuffer_.setup({
        .width = 2000,
        .height = 2000,
        .colorAttachments = {
            {0, render::FramebufferAttachment{
                .texture = &sceneOutputTexture_,
                .format = render::TextureFormat::RGBA32F,
            }},
        },
        .depthAttachment = render::FramebufferDepthAttachment{
            .texture = &sceneDepthTexture_,
            .format = render::DepthAttachmentFormat::DEPTH24_STENCIL8,
        },
    });
    if (status != render::Framebuffer::Status::COMPLETE) {
        std::cerr << "Framebuffer failed; status: " << status << "\n";
        return false;
    }

    return true;
}

void SceneRenderer::sync(const std::vector<Body>& bodies, std::span<const Plane> planes) {
    instances_.resize(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) {
        glm::mat4 model{1.f};
        model = glm::translate(model, bodies[i].position);
        model = glm::scale(model, glm::vec3{2.f * bodies[i].radius});
        instances_[i] = InstanceData{.color = bodies[i].color, .model = model};
    }
    instanceBuffer_->setData(instances_);

    planeInstances_.clear();
    for (const Plane& plane : planes) {
        if (!plane.enabled) continue;
        planeInstances_.push_back(InstanceData{
            .color = kPlaneColor,
            .model = planeInstanceMatrix(plane, kPlaneVisualHalfExtent),
        });
    }
    planeInstanceBuffer_->setData(planeInstances_);
}

void SceneRenderer::render(const render::Camera& camera, const SceneRenderOptions& options) {
    const auto windowSize = render::window::getSize();

    SceneData sceneData;
    sceneData.viewPos = camera.getPosition();
    sceneData.view = camera.getViewMatrix();
    sceneData.projection = glm::perspective(
        glm::radians(camera.fov),
        static_cast<float>(windowSize.x) / static_cast<float>(windowSize.y),
        camera.minDist, camera.maxDist
    );
    ubo_.setData(sceneData);

    sceneFramebuffer_.bind();
    meshShader_.use();
    mesh_.render();

    if (!planeInstances_.empty()) {
        glDisable(GL_CULL_FACE);
        planeMesh_.render();
        glEnable(GL_CULL_FACE);
    }

    if (options.applyBloom) {
        const auto& bloomTexture = bloomEffect_.apply(sceneOutputTexture_);
        render::Framebuffer::bindDefault(windowSize.x, windowSize.y);
        util::renderScreenQuadTexture(bloomTexture);
    } else {
        render::Framebuffer::bindDefault(windowSize.x, windowSize.y);
        util::renderScreenQuadTexture(sceneOutputTexture_);
    }
}
