#version 330 core
out vec4 FragColor;

in vec4 vertexColor;
in vec2 TexCoords;

struct Material {
    sampler2D diffuse;
};
uniform Material material;
uniform bool useTextureColorOnly;
uniform bool textureOn = true;
uniform vec3 objectColor = vec3(1.0, 1.0, 1.0);

vec4 getDiffuseMap() {
    return textureOn ? texture(material.diffuse, TexCoords) : vec4(objectColor, 1.0);
}

void main()
{
    if(useTextureColorOnly) {
        FragColor = getDiffuseMap();
    } else {
        FragColor = vertexColor * getDiffuseMap();
    }
}
