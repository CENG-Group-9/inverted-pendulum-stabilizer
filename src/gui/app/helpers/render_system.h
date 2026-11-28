#ifndef SYSTEMS_H
#define SYSTEMS_H

#include <vector>
#include "entity.h"
#include "camera.h"
#include "light_manager.h"
#include "model.h"

class RenderSystem {
public:
    void update(std::vector<Entity>& entities, Camera& camera, 
                const LightManager& lights, float aspectRatio) {
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom),
                                                 aspectRatio, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();

        for (auto& e : entities) {
            if (e.usesBlending) {
              glEnable(GL_BLEND);
              glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            }
            e.shader->use();
            e.shader->setVec3("light_color", e.color);
            e.shader->setMat4("projection", projection);
            e.shader->setMat4("view", view);
            e.shader->setMat4("model", e.getModelMatrix());
            e.shader->setVec3("viewPos", camera.Position);
            lights.ApplyLights(*e.shader);
            e.model.Draw(*e.shader);
            if (e.usesBlending) {
              glDisable(GL_BLEND);
            }
        }
    }
};
#endif
