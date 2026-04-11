import os

with open('main.cpp', 'r') as f:
    text = f.read()

if 'bool textureOn =' not in text:
    text = text.replace('bool useTextureColorOnly = false;', 'bool useTextureColorOnly = false;\nbool textureOn = true;\nbool keyR_pressed = false;')

if 'DO_TOGGLE(GLFW_KEY_R, textureOn' not in text:
    text = text.replace('DO_TOGGLE(GLFW_KEY_T, useTextureColorOnly, keyT_pressed)', 'DO_TOGGLE(GLFW_KEY_T, useTextureColorOnly, keyT_pressed)\n    DO_TOGGLE(GLFW_KEY_R, textureOn, keyR_pressed)')

text = text.replace('if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) mainCamera.ProcessKeyboard(DOWN, deltaTime);', 'if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) mainCamera.ProcessKeyboard(DOWN, deltaTime);')

text = text.replace('shader.setBool("useTextureColorOnly", useTextureColorOnly);', 'shader.setBool("useTextureColorOnly", useTextureColorOnly);\n    shader.setBool("textureOn", textureOn);')

with open('main.cpp', 'w') as f:
    f.write(text)

with open('fragmentShaderForPhongShading.fs', 'r') as f:
    fs_p = f.read()
fs_p = fs_p.replace('if (useTextureColorOnly) {\n        FragColor = texture(material.diffuse, TexCoords);\n        return;\n    }', 'if (useTextureColorOnly) {\n        FragColor = getDiffuseMap();\n        return;\n    }')
with open('fragmentShaderForPhongShading.fs', 'w') as f:
    f.write(fs_p)

with open('fragmentShaderForGouraudShading.fs', 'r') as f:
    fs_g = f.read()

if 'uniform bool textureOn;' not in fs_g:
    fs_g = fs_g.replace('uniform bool useTextureColorOnly;', 'uniform bool useTextureColorOnly;\nuniform bool textureOn = true;\n\nvec4 getDiffuseMap() {\n    return textureOn ? texture(material.diffuse, TexCoords) : vec4(1.0);\n}')
    fs_g = fs_g.replace('texture(material.diffuse, TexCoords)', 'getDiffuseMap()')
    with open('fragmentShaderForGouraudShading.fs', 'w') as f:
        f.write(fs_g)

print('Patch applied successfully')
