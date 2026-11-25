#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_tex_coords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 world_position;
out vec3 world_normal;
out vec2 tex_coords;

void main()
{
    vec4 world = model * vec4(vertex_position, 1.0);
    world_position = world.xyz;
    world_normal = mat3(transpose(inverse(model))) * vertex_normal;
    tex_coords = vertex_tex_coords;
    gl_Position = projection * view * world;
}