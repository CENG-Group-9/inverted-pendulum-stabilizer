#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "model.h"
#include "shader.h"

class GameObject {
public:
    // Transform
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 rotation = glm::vec3(0.0f);
    glm::vec3 scale    = glm::vec3(1.0f);

    // Identity
    std::string name;

    // Collision
    bool isSolid = false;
    glm::vec3 aabbMin = glm::vec3(-0.5f);
    glm::vec3 aabbMax = glm::vec3( 0.5f);

    GameObject(const std::string& modelPath, const std::string& name = "GameObject")
        : model(modelPath), name(name) {}

    void Draw(Shader& shader) {
        shader.setMat4("model", getModelMatrix());
        model.Draw(shader);
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

    // Returns world-space AABB
    std::pair<glm::vec3, glm::vec3> getWorldAABB() const {
        return {
            position + aabbMin * scale,
            position + aabbMax * scale
        };
    }

    bool collidesWith(const GameObject& other) const {
        if (!isSolid || !other.isSolid) return false;
        auto [minA, maxA] = getWorldAABB();
        auto [minB, maxB] = other.getWorldAABB();
        return (minA.x <= maxB.x && maxA.x >= minB.x) &&
               (minA.y <= maxB.y && maxA.y >= minB.y) &&
               (minA.z <= maxB.z && maxA.z >= minB.z);
    }

private:
    Model model;
};
#endif
