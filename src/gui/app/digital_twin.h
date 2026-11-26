#pragma once
#include "helpers/entity.h"
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
#include "system_status.h"

namespace pendulum::gui {

class DigitalTwin {
  public:
    DigitalTwin()
        : shader_(PENDULUM_SHADER_DIR "/model.vert", PENDULUM_SHADER_DIR "/model.frag"),
          camera_(glm::vec3(0.0F, 0.2F, 0.0F), 1.8F, glm::radians(15.0F), glm::radians(15.0F)) {
        Mesh::initWhiteTexture();

        lights_.setDirLight(glm::vec3(-0.4F, -1.0F, -0.6F), glm::vec3(0.8F, 0.8F, 0.8F));
        lights_.addPointLight(glm::vec3(0.0, -0.5, 0.0), glm::vec3(0.8, 0.8, 0.8));

        entities_.reserve(3);

        Entity& cart = add_entity("cart", PENDULUM_ASSET_DIR "/pendulum/cart.obj");
        Entity& rod = add_entity("rod", PENDULUM_ASSET_DIR "/pendulum/rod.obj");
        add_entity("rail", PENDULUM_ASSET_DIR "/pendulum/rail.obj");

        // The cart is modeled as a cube centered on its origin, so its half height is the
        // distance from the rail plane to the cart center.
        cart.scale = glm::vec3(1.0F);
        cart.position = glm::vec3(0.0F, 0.0F, 0.0F);

        // The rod is modeled hanging down from its top cap, which is the hinge point.
        const glm::vec3 rod_center_meters = (rod.aabbMin + rod.aabbMax) * 0.5F;
        rod.pivot_meters = glm::vec3(rod_center_meters.x, rod.aabbMax.y, rod_center_meters.z);

        update_pose(SystemStatus{});
    }

    OrbitCamera& camera() { return camera_; }
    const OrbitCamera& camera() const { return camera_; }

    // Returns the existing entity when the name is already registered.
    Entity& add_entity(const std::string& name, const std::string& model_path) {
        const auto existing = entity_index_by_name_.find(name);
        if (existing != entity_index_by_name_.end()) {
            return entities_[existing->second];
        }

        entities_.emplace_back(model_path, &shader_, name);
        entity_index_by_name_.emplace(name, entities_.size() - 1);
        return entities_.back();
    }

    Entity* find_entity(const std::string& name) {
        const auto index = entity_index_by_name_.find(name);
        if (index == entity_index_by_name_.end()) {
            return nullptr;
        }
        return &entities_[index->second];
    }

    const Entity* find_entity(const std::string& name) const {
        const auto index = entity_index_by_name_.find(name);
        if (index == entity_index_by_name_.end()) {
            return nullptr;
        }
        return &entities_[index->second];
    }

    // Maps SI telemetry onto the scene: the rail runs along Z and the rod tilts about X.
    void update_pose(const SystemStatus& status) {
        Entity* cart = find_entity("cart");
        Entity* rod = find_entity("rod");
        if (cart == nullptr || rod == nullptr) {
            return;
        }

        const float cart_position_meters = static_cast<float>(status.cart_position_meters);
        cart->position.z = cart_position_meters;

        const float cart_top_height_meters = cart->position.y + cart->aabbMax.y * cart->scale.y;
        rod->position = glm::vec3(0.0F, cart_top_height_meters, cart_position_meters);
        rod->rotation.x =
            kRodUprightRotationDegrees + glm::degrees(static_cast<float>(status.rod_angle_radians));
    }

    void render(int framebuffer_width_pixels, int framebuffer_height_pixels, const SystemStatus& current_status) {
        if (framebuffer_width_pixels <= 0 || framebuffer_height_pixels <= 0) {
            return;
        }

        glViewport(0, 0, framebuffer_width_pixels, framebuffer_height_pixels);
        glEnable(GL_DEPTH_TEST);

        const float aspect_ratio = static_cast<float>(framebuffer_width_pixels) /
                                   static_cast<float>(framebuffer_height_pixels);
        update_pose(current_status);
        render_system_.update(entities_, camera_, lights_, aspect_ratio);
    }

  private:
    static constexpr float kRodUprightRotationDegrees = 0.0F;

    Shader shader_;
    OrbitCamera camera_;
    LightManager lights_;
    RenderSystem render_system_;
    std::vector<Entity> entities_;
    std::unordered_map<std::string, std::size_t> entity_index_by_name_;
};

} // namespace pendulum::gui
