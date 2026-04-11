import os

with open('main.cpp', 'r') as f:
    text = f.read()

# Make ambientOn independent of masterLightOn (so the user can still see Step 3 blending in the dark)
text = text.replace('shader.setBool("ambientOn", masterLightOn && ambientOn);', 'shader.setBool("ambientOn", ambientOn);')

with open('main.cpp', 'w') as f:
    f.write(text)

print('Ambient decoupled from masterLight')
