#include "render/scene_renderer.hpp"

#include <iostream>

#include <glm/gtc/matrix_transform.hpp>

#include "tmig/render/framebuffer.hpp"
#include "tmig/render/window.hpp"
#include "tmig/util/postprocessing.hpp"
#include "tmig/util/resources.hpp"

using namespace tmig;

SceneRenderer::SceneRenderer() {
    ubo_.bindTo(0);
}

SceneRenderer::~SceneRenderer() {
    delete vertexBuffer_;
    delete instanceBuffer_;
    delete indexBuffer_;
}

bool SceneRenderer::init() {
    if (!meshShader_.compileFromFiles(
        util::getResourcePath("shaders/instanced.vert"),
        util::getResourcePath("shaders/instanced_bloom.frag")
    )) {
        std::cerr << "Failed loading instanced shader\n";
        return false;
    }

    std::vector<util::GeneralVertex> vertices;
    std::vector<uint32_t> indices;
    util::generateSphereMesh([&](auto v) { vertices.push_back(v); }, indices, 15);

    vertexBuffer_ = new render::DataBuffer<util::GeneralVertex>;
    instanceBuffer_ = new render::DataBuffer<InstanceData>;
    indexBuffer_ = new render::DataBuffer<uint32_t>;
    vertexBuffer_->setData(vertices);
    indexBuffer_->setData(indices);

    mesh_.setAttributes({
        render::VertexAttributeType::FLOAT3,
        render::VertexAttributeType::FLOAT3,
        render::VertexAttributeType::FLOAT2,
    }, {
        render::VertexAttributeType::FLOAT4,
        render::VertexAttributeType::MAT4x4,
    });
    mesh_.setInstanceBuffer(instanceBuffer_);
    mesh_.setIndexBuffer(indexBuffer_);
    mesh_.setVertexBuffer(vertexBuffer_);

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

void SceneRenderer::sync(const std::vector<Body>& bodies) {
    instances_.resize(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) {
        glm::mat4 model{1.f};
        model = glm::translate(model, bodies[i].position);
        model = glm::scale(model, glm::vec3{2.f * bodies[i].radius});
        instances_[i] = InstanceData{.color = bodies[i].color, .model = model};
    }

    instanceBuffer_->setData(instances_);
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

    if (options.applyBloom) {
        const auto& bloomTexture = bloomEffect_.apply(sceneOutputTexture_);
        render::Framebuffer::bindDefault(windowSize.x, windowSize.y);
        util::renderScreenQuadTexture(bloomTexture);
    } else {
        render::Framebuffer::bindDefault(windowSize.x, windowSize.y);
        util::renderScreenQuadTexture(sceneOutputTexture_);
    }
}
