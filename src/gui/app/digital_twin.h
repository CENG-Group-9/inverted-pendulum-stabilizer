#pragma once
#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "helpers/light_manager.h"
#include "helpers/orbit_camera.h"
#include "helpers/render_system.h"
#include "helpers/shader.h"

namespace pendulum::gui {

class DigitalTwin {
  public:
    DigitalTwin()
        : shader_(PENDULUM_SHADER_DIR "/model.vert", PENDULUM_SHADER_DIR "/model.frag"),
          camera_(glm::vec3(0.0F, 0.0F, -0.2F)) 
    {
        Mesh::initBlackTexture();

        lights_.setDirLight(glm::vec3(-0.4F, -1.0F, -0.6F), glm::vec3(0.8F, 0.8F, 0.8F));

        entities_.reserve(1);

        Entity& pyramid_monkey =
            add_entity("pyramid_monkey", PENDULUM_ASSET_DIR "/pyramid_monkey/pyramid_monkey.obj");

        pyramid_monkey.scale = glm::vec3(0.7F);
        pyramid_monkey.position = glm::vec3(0.0F, -0.1F, 0.0F);
    }

    [[nodiscard]] OrbitCamera& camera() { return camera_; }
    [[nodiscard]] const OrbitCamera& camera() const { return camera_; }

    // Returns the existing entity when the name is already registered.
    Entity& add_entity(const std::string& name, const std::string& model_path) 
    {
        const auto existing = entity_index_by_name_.find(name);
        if (existing != entity_index_by_name_.end()) {
            return entities_[existing->second];
        }

        entities_.emplace_back(model_path, &shader_, name);
        entity_index_by_name_.emplace(name, entities_.size() - 1);
        return entities_.back();
    }

    [[nodiscard]] Entity* find_entity(const std::string& name) 
    {
        const auto index = entity_index_by_name_.find(name);
        if (index == entity_index_by_name_.end()) {
            return nullptr;
        }
        return &entities_[index->second];
    }

    [[nodiscard]] const Entity* find_entity(const std::string& name) const 
    {
        const auto index = entity_index_by_name_.find(name);
        if (index == entity_index_by_name_.end()) {
            return nullptr;
        }
        return &entities_[index->second];
    }

    void render(int framebuffer_width_pixels, int framebuffer_height_pixels, double dt) 
    {
        if (framebuffer_width_pixels <= 0 || framebuffer_height_pixels <= 0) 
        {
            return;
        }

        glViewport(0, 0, framebuffer_width_pixels, framebuffer_height_pixels);
        glEnable(GL_DEPTH_TEST);

        const float aspect_ratio = static_cast<float>(framebuffer_width_pixels) /
                                   static_cast<float>(framebuffer_height_pixels);

        constexpr float kSpinSpeedDegreesPerSecond = 60.0f;
        Entity* pyramid_monkey = find_entity("pyramid_monkey");
        pyramid_monkey->rotation.y = std::fmod(pyramid_monkey->rotation.y + kSpinSpeedDegreesPerSecond * dt, 360.0F);
         
        render_system_.update(entities_, camera_, lights_, aspect_ratio);
    }

  private:
    Shader shader_;
    OrbitCamera camera_;
    LightManager lights_;
    RenderSystem render_system_;
    std::vector<Entity> entities_;
    std::unordered_map<std::string, std::size_t> entity_index_by_name_;
};

} // namespace pendulum::gui
