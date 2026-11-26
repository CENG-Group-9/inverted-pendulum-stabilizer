#ifndef MESH_H
#define MESH_H

#include "glad/gl.h" // holds all OpenGL type declarations

#include <assimp/material.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "glm/detail/type_vec.hpp"
#include "shader.h"

#include <string>
#include <vector>
using namespace std;

#define MAX_BONE_INFLUENCE 4

struct Vertex {
  // position
  glm::vec3 Position;
  // normal
  glm::vec3 Normal;
  // texCoords
  glm::vec2 TexCoords;
  // tangent
  glm::vec3 Tangent;
  // bitangent
  glm::vec3 Bitangent;
  // bone indexes which will influence this vertex
  int m_BoneIDs[MAX_BONE_INFLUENCE];
  // weights from each bone
  float m_Weights[MAX_BONE_INFLUENCE];
};

struct Texture {
  unsigned int id;
  string type;
  string path;
};

struct SurfaceMaterial {
  glm::vec3 ambient_color{1.0f};
  glm::vec3 diffuse_color{1.0f};
  glm::vec3 specular_color{0.0f};
  float shininess{32.0f};
};

class Mesh {
public:
  // mesh Data
  vector<Vertex> vertices;
  vector<unsigned int> indices;
  vector<Texture> textures;
  SurfaceMaterial material;
  unsigned int VAO;

  // constructor
  Mesh(vector<Vertex> vertices, vector<unsigned int> indices,
       vector<Texture> textures, SurfaceMaterial surface_material = {}) {
    this->vertices = vertices;
    this->indices = indices;
    this->textures = textures;
    this->material = surface_material;

    // now that we have all the required data, set the vertex buffers and its
    // attribute pointers.
    setupMesh();
  }

  // render the mesh
  void Draw(Shader &shader) {
    int diffuseSlot = -1;
    int specularSlot = -1;
    for (unsigned int i = 0; i < textures.size(); i++) {
      glActiveTexture(GL_TEXTURE0 + i);
      glBindTexture(GL_TEXTURE_2D, textures[i].id);
      if (textures[i].type == "texture_diffuse")
        diffuseSlot = static_cast<int>(i);
      else if (textures[i].type == "texture_specular")
        specularSlot = static_cast<int>(i);
    }

    int nextSlot = static_cast<int>(textures.size());
    if (diffuseSlot == -1) {
      diffuseSlot = nextSlot++;
      glActiveTexture(GL_TEXTURE0 + diffuseSlot);
      glBindTexture(GL_TEXTURE_2D, whiteTexture);
    }
    if (specularSlot == -1) {
      specularSlot = nextSlot++;
      glActiveTexture(GL_TEXTURE0 + specularSlot);
      glBindTexture(GL_TEXTURE_2D, whiteTexture);
    }

    shader.setInt("material.diffuse", diffuseSlot);
    shader.setInt("material.specular", specularSlot);
    shader.setVec3("material.ambient_color", material.ambient_color);
    shader.setVec3("material.diffuse_color", material.diffuse_color);
    shader.setVec3("material.specular_color", material.specular_color);
    shader.setFloat("material.shininess", material.shininess);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()),
                   GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    glActiveTexture(GL_TEXTURE0);
  }

  static void initWhiteTexture() {
    if (whiteTextureInitialized)
      return;
    unsigned char white[] = {255, 255, 255, 255};
    glGenTextures(1, &whiteTexture);
    glBindTexture(GL_TEXTURE_2D, whiteTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 white);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    whiteTextureInitialized = true;
  }

private:
  // render data
  unsigned int VBO, EBO;

  inline static unsigned int whiteTexture;
  inline static bool whiteTextureInitialized;

  // initializes all the buffer objects/arrays
  void setupMesh() {
    // create buffers/arrays
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // A great thing about structs is that their memory layout is sequential for
    // all its items. The effect is that we can simply pass a pointer to the
    // struct and it translates perfectly to a glm::vec3/2 array which again
    // translates to 3/2 floats which translates to a byte array.
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex),
                 &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 &indices[0], GL_STATIC_DRAW);

    // set the vertex attribute pointers
    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, Normal));
    // vertex texture coords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, TexCoords));
    // vertex tangent
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, Tangent));
    // vertex bitangent
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, Bitangent));
    // ids
    glEnableVertexAttribArray(5);
    glVertexAttribIPointer(5, 4, GL_INT, sizeof(Vertex),
                           (void *)offsetof(Vertex, m_BoneIDs));

    // weights
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, m_Weights));
    glBindVertexArray(0);
  }
};
#endif
