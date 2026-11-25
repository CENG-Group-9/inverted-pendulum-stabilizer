#ifndef LIGHT_MANAGER_H
#define LIGHT_MANAGER_H

#include <glm/glm.hpp>
#include <vector>
#include <string>
#include "game_object.h"
#include "shader.h"

struct PointLight {
    glm::vec3 position  = glm::vec3(0.0f);
    glm::vec3 ambient   = glm::vec3(0.05f);
    glm::vec3 diffuse   = glm::vec3(0.8f);
    glm::vec3 specular  = glm::vec3(1.0f);
    float constant      = 1.0f;
    float linear        = 0.09f;
    float quadratic     = 0.032f;
};

struct DirLight {
    glm::vec3 direction = glm::vec3(0.0f, -1.0f, 0.0f);
    glm::vec3 ambient   = glm::vec3(0.05f);
    glm::vec3 diffuse   = glm::vec3(0.5f);
    glm::vec3 specular  = glm::vec3(0.5f);
};

class LightManager {
public:
    DirLight directionalLight;
    std::vector<PointLight> pointLights;

    // Fluent builders — return *this so you can chain calls if you want
    LightManager& setDirLight(glm::vec3 direction,
                               glm::vec3 diffuse,
                               glm::vec3 ambient  = glm::vec3(0.05f),
                               glm::vec3 specular = glm::vec3(0.5f)) {
        directionalLight.direction = direction;
        directionalLight.diffuse   = diffuse;
        directionalLight.ambient   = ambient;
        directionalLight.specular  = specular;
        return *this;
    }

    LightManager& addPointLight(glm::vec3 position,
                                 glm::vec3 diffuse,
                                 glm::vec3 ambient   = glm::vec3(0.05f),
                                 glm::vec3 specular  = glm::vec3(1.0f),
                                 float linear        = 0.09f,
                                 float quadratic     = 0.032f,
                                 float constant      = 1.0) {
        PointLight p;
        p.position  = position;

        p.constant  = constant;
        p.linear    = linear;
        p.quadratic = quadratic;

        p.diffuse   = diffuse;
        p.ambient   = ambient;
        p.specular  = specular;

        pointLights.push_back(p);
        return *this;
    }

    void ApplyLights(const Shader& shader) const {
        // Directional light
        shader.setVec3("dirLight.direction", directionalLight.direction);
        shader.setVec3("dirLight.ambient",   directionalLight.ambient);
        shader.setVec3("dirLight.diffuse",   directionalLight.diffuse);
        shader.setVec3("dirLight.specular",  directionalLight.specular);

        // Point lights — also send the count so the shader can loop correctly
        shader.setInt("numPointLights", (int)pointLights.size());
        for (unsigned int i = 0; i < pointLights.size(); i++) {
            std::string b = "pointLights[" + std::to_string(i) + "].";
            shader.setVec3 (b + "position",  pointLights[i].position);
            shader.setVec3 (b + "ambient",   pointLights[i].ambient);
            shader.setVec3 (b + "diffuse",   pointLights[i].diffuse);
            shader.setVec3 (b + "specular",  pointLights[i].specular);
            shader.setFloat(b + "constant",  pointLights[i].constant);
            shader.setFloat(b + "linear",    pointLights[i].linear);
            shader.setFloat(b + "quadratic", pointLights[i].quadratic);
        }
    }

    void DrawLights(GameObject light_object, Shader& shader) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      for (unsigned int i = 0; i < pointLights.size(); i++) {
        light_object.position = pointLights[i].position;
        // glm::vec3 light_color;
        // light_color.x = pointLights[i].diffuse.x;
        // light_color.y = pointLights[i].diffuse.y;
        // light_color.z = pointLights[i].diffuse.z;
        shader.setVec3("light_color", pointLights[i].diffuse);

        light_object.Draw(shader);
      }
      glDisable(GL_BLEND);
    }
};

#endif
