#ifndef ENTITY_H
#define ENTITY_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include "model.h"
#include "shader.h"

struct Entity {
    std::string name;

    // Transform
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 rotation = glm::vec3(0.0f);
    glm::vec3 scale    = glm::vec3(1.0f);

    // Rendering
    Model  model;
    Shader* shader = nullptr;
    glm::vec3 color = glm::vec3(1.0f);

    // Collision
    bool isSolid   = false;
    bool isRoom    = false;
    glm::vec3 aabbInnerMin = glm::vec3(0.0f);
    glm::vec3 aabbInnerMax = glm::vec3(0.0f);
    
    // for point light renders
    bool usesBlending = false;

    glm::vec3 aabbMin = glm::vec3(0.0f);
    glm::vec3 aabbMax = glm::vec3(0.0f);

    Entity(const std::string& modelPath, Shader* shader, const std::string& name = "Entity")
        : model(modelPath), shader(shader), name(name) {}

    void setSolid(bool solid, float wallThickness = 1.0f) {
        isSolid = solid;
        if (solid) {
            computeAABB();
        }
    }

    glm::mat4 getModelMatrix() const {
        glm::mat4 mat = glm::mat4(1.0f);
        mat = glm::translate(mat, position);
        mat = glm::rotate(mat, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
        mat = glm::rotate(mat, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        mat = glm::rotate(mat, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
        mat = glm::scale(mat, scale);
        return mat;
    }

    std::pair<glm::vec3, glm::vec3> getWorldAABB() const {
        return {
            position + aabbMin * scale,
            position + aabbMax * scale
        };
    }

    void computeAABB(float wallThickness = 1.0f) {
        aabbMin = glm::vec3(FLT_MAX);
        aabbMax = glm::vec3(-FLT_MAX);
        for (auto& mesh : model.meshes) {
            for (auto& vertex : mesh.vertices) {
                aabbMin = glm::min(aabbMin, vertex.Position);
                aabbMax = glm::max(aabbMax, vertex.Position);
            }
        }
        if (isRoom) {
            aabbInnerMin = aabbMin + glm::vec3(wallThickness);
            aabbInnerMax = aabbMax - glm::vec3(wallThickness);
        }
    }
};
#endif
