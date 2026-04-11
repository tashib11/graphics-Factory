import os

with open('main.cpp', 'r') as f:
    text = f.read()

# Replace setVec3 for material.diffuse with objectColor
text = text.replace('setVec3("material.diffuse"', 'setVec3("objectColor"')

with open('main.cpp', 'w') as f:
    f.write(text)

# Now update the shaders
with open('fragmentShaderForPhongShading.fs', 'r') as f:
    fs_p = f.read()

if 'uniform vec3 objectColor;' not in fs_p:
    fs_p = fs_p.replace('uniform bool textureOn = true;', 'uniform bool textureOn = true;\nuniform vec3 objectColor = vec3(1.0, 1.0, 1.0);')
    fs_p = fs_p.replace('return textureOn ? texture(material.diffuse, TexCoords) : vec4(1.0);', 'return textureOn ? texture(material.diffuse, TexCoords) : vec4(objectColor, 1.0);')
    with open('fragmentShaderForPhongShading.fs', 'w') as f:
        f.write(fs_p)

with open('fragmentShaderForGouraudShading.fs', 'r') as f:
    fs_g = f.read()

if 'uniform vec3 objectColor;' not in fs_g:
    fs_g = fs_g.replace('uniform bool textureOn = true;', 'uniform bool textureOn = true;\nuniform vec3 objectColor = vec3(1.0, 1.0, 1.0);')
    fs_g = fs_g.replace('return textureOn ? texture(material.diffuse, TexCoords) : vec4(1.0);', 'return textureOn ? texture(material.diffuse, TexCoords) : vec4(objectColor, 1.0);')
    with open('fragmentShaderForGouraudShading.fs', 'w') as f:
        f.write(fs_g)

with open('task.md', 'w') as f:
    f.write('''# Task Checklist
- [x] Step 1: Implement Surface Color mapping (`objectColor`) and hook up to R key
- [ ] Step 2: Confirm `T` texturing without surface color works as requested
- [ ] Step 3: Implement `P` shader transition correctly using forced ambient lighting while master light is off
''')

print('Patch applied')
