#ifndef POINT_LIGHT_H
#define POINT_LIGHT_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include "shader.h"

class PointLight {
public:
    glm::vec3 position;

    float constant;
    float linear;
    float quadratic;

    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;

    bool isOn;

    PointLight(glm::vec3 pos, glm::vec3 amb, glm::vec3 diff, glm::vec3 spec, float c, float l, float q) {
        position = pos;
        ambient = amb;
        diffuse = diff;
        specular = spec;
        constant = c;
        linear = l;
        quadratic = q;
        isOn = true;
    }

    void setUpPointLight(Shader& shader, const std::string& prefix) {
        if (!isOn) {
            shader.setVec3(prefix + ".ambient", glm::vec3(0.0f));
            shader.setVec3(prefix + ".diffuse", glm::vec3(0.0f));
            shader.setVec3(prefix + ".specular", glm::vec3(0.0f));
        }
        else {
            shader.setVec3(prefix + ".ambient", ambient);
            shader.setVec3(prefix + ".diffuse", diffuse);
            shader.setVec3(prefix + ".specular", specular);
        }

        shader.setVec3(prefix + ".position", position);
        shader.setFloat(prefix + ".constant", constant);
        shader.setFloat(prefix + ".linear", linear);
        shader.setFloat(prefix + ".quadratic", quadratic);
    }

    void turnOn() { isOn = true; }
    void turnOff() { isOn = false; }
    void toggle() { isOn = !isOn; }
};

#endif
