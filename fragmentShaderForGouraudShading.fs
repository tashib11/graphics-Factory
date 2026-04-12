#version 330 core
out vec4 FragColor;

in vec4 vertexColor;
in vec2 TexCoords;
in vec3 FragPos;

struct Material {
    sampler2D diffuse;
};
uniform Material material;
uniform bool useTextureColorOnly;
uniform bool textureOn = true;
uniform vec3 objectColor = vec3(1.0, 1.0, 1.0);

// ── Texture-scroll support ──────────────────────────────────────────────────
uniform bool  useTexOffset;
uniform vec2  texOffset;
uniform vec2  texScale = vec2(1.0, 1.0);
uniform vec3  viewPos;
uniform bool  isSkybox = false;

vec2 getUV() {
    if (isSkybox) {
        vec3 d = normalize(FragPos - viewPos);
        vec2 uv = vec2(atan(d.z, d.x), asin(d.y));
        uv *= vec2(0.1591549, 0.3183098);
        return uv + 0.5;
    }
    vec2 st = TexCoords * texScale;
    return useTexOffset ? st + texOffset : st;
}

vec4 getDiffuseMap() {
    return textureOn ? texture(material.diffuse, getUV()) : vec4(objectColor, 1.0);
}

void main()
{
    if(useTextureColorOnly) {
        FragColor = getDiffuseMap();
    } else {
        FragColor = vertexColor * getDiffuseMap();
    }
}
