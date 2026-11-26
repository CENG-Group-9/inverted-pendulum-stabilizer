#version 330 core

#define MAX_POINT_LIGHTS 8

struct Material {
    sampler2D diffuse;
    sampler2D specular;
    vec3 ambient_color;
    vec3 diffuse_color;
    vec3 specular_color;
    float shininess;
};

struct DirLight {
    vec3 direction;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

struct PointLight {
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float constant;
    float linear;
    float quadratic;
};

in vec3 world_position;
in vec3 world_normal;
in vec2 tex_coords;

uniform vec3 viewPos;
uniform Material material;
uniform DirLight dirLight;
uniform PointLight pointLights[MAX_POINT_LIGHTS];
uniform int numPointLights;

out vec4 fragment_color;

vec3 calc_dir_light(DirLight light, vec3 normal, vec3 view_dir)
{
    vec3 light_dir = normalize(-light.direction);
    float diffuse_strength = max(dot(normal, light_dir), 0.0);
    vec3 halfway = normalize(light_dir + view_dir);
    float specular_strength = pow(max(dot(normal, halfway), 0.0), material.shininess);

    vec3 diffuse_sample = vec3(texture(material.diffuse, tex_coords));
    vec3 specular_sample = vec3(texture(material.specular, tex_coords));

    vec3 diffuse_color = diffuse_sample * material.diffuse_color;
    vec3 specular_color = specular_sample * material.specular_color;

    return light.ambient * material.ambient_color * diffuse_color
         + light.diffuse * diffuse_strength * diffuse_color
         + light.specular * specular_strength * specular_color;
}

vec3 calc_point_light(PointLight light, vec3 normal, vec3 view_dir)
{
    vec3 light_dir = normalize(light.position - world_position);
    float diffuse_strength = max(dot(normal, light_dir), 0.0);
    vec3 halfway = normalize(light_dir + view_dir);
    float specular_strength = pow(max(dot(normal, halfway), 0.0), material.shininess);

    float distance = length(light.position - world_position);
    float attenuation = 1.0 / (light.constant
                             + light.linear * distance
                             + light.quadratic * distance * distance);

    vec3 diffuse_sample = vec3(texture(material.diffuse, tex_coords));
    vec3 specular_sample = vec3(texture(material.specular, tex_coords));

    vec3 diffuse_color = diffuse_sample * material.diffuse_color;
    vec3 specular_color = specular_sample * material.specular_color;

    return attenuation * (light.ambient * material.ambient_color * diffuse_color
                        + light.diffuse * diffuse_strength * diffuse_color
                        + light.specular * specular_strength * specular_color);
}

void main()
{
    vec3 normal = normalize(world_normal);
    vec3 view_dir = normalize(viewPos - world_position);

    vec3 result = calc_dir_light(dirLight, normal, view_dir);
    for (int i = 0; i < numPointLights && i < MAX_POINT_LIGHTS; ++i) {
        result += calc_point_light(pointLights[i], normal, view_dir);
    }

    fragment_color = vec4(result, 1.0);
}