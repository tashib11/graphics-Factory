


#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "shader.h"
#include "camera.h"

#include <iostream>
#include <vector>
#include <cstdio>
#include <direct.h>

// window settings
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;

// cameras for 4 viewports
Camera mainCamera(glm::vec3(0.0f, 5.0f, 15.0f));
Camera birdEyeCamera(glm::vec3(0.0f, 20.0f, 0.1f));
Camera followCamera(glm::vec3(0.0f, 5.0f, 15.0f));
Camera frontCamera(glm::vec3(0.0f, 2.0f, 10.0f));

float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// Toggles state
bool dirLightOn = true;
bool pointLightOn = true;
bool spotLightOn = true;
bool ambientOn = true;
bool diffuseOn = true;
bool specularOn = true;
bool masterLightOn = true;
bool mainLightOn = true;
bool usePhong = true;
bool fanOn = false;
bool singleViewport = false;
bool useTextureColorOnly = false;

// Debounce state
bool key1_pressed = false, key2_pressed = false, key3_pressed = false;
bool key5_pressed = false, key6_pressed = false, key7_pressed = false;
bool keyL_pressed = false, keyM_pressed = false, keyG_pressed = false;
bool key8_pressed = false, key9_pressed = false, keyV_pressed = false;
bool keyT_pressed = false;
bool keyP_pressed = false;

// Animations
float fanAngle = 0.0f;
float robotBaseAngle = 0.0f;
float robotElbowAngle = 0.0f;
float bindRotAngle = 0.0f;
float armReachAngle = -30.0f;
float gripperSpread = 0.3f; // gripper finger spread (0.1=closed, 0.7=open)

struct ConveyorLine {
    glm::vec3 start;
    glm::vec3 end;
    bool isHorizontal;
};
std::vector<ConveyorLine> lines = {
    // Horizontal lines (along X axis) — LOWER level y=0.8
    {{-44, 0.8f, -20}, {44, 0.8f, -20}, true},
    {{-44, 0.8f, -10}, {44, 0.8f, -10}, true},
    {{-44, 0.8f,   0}, {44, 0.8f,   0}, true},
    {{-44, 0.8f,  10}, {44, 0.8f,  10}, true},
    {{-44, 0.8f,  20}, {44, 0.8f,  20}, true},
    // Vertical lines (along Z axis) — UPPER level y=4.5 to pass clearly OVER horizontal tunnel chambers
    {{-20, 4.5f, -44}, {-20, 4.5f, 44}, false},
    {{-10, 4.5f, -44}, {-10, 4.5f, 44}, false},
    {{  0, 4.5f, -44}, {  0, 4.5f, 44}, false},
    {{ 10, 4.5f, -44}, { 10, 4.5f, 44}, false},
    {{ 20, 4.5f, -44}, { 20, 4.5f, 44}, false}
};

enum BoxStage { RAW = 0, PAINTED = 1, BOUND = 2 };
enum BoxState { ON_BELT, WAITING_FOR_PICKUP, BEING_PICKED, ON_SHELF };

struct GridBox {
    int lineIndex;
    float distance;
    BoxStage stage;
    BoxState state = ON_BELT;
    glm::vec3 worldPos = glm::vec3(0.0f); // used when off-belt
    int shelfTier = 0;
    int shelfSlot = 0;
    float pickTimer = 0.0f;
    float onShelfT = 0.0f;
};
std::vector<GridBox> gridBoxes;

// Shelf placement tracking
const int SHELF_TIERS = 5;
const int SHELF_SLOTS = 8;
bool shelfOccupied[10][SHELF_TIERS][SHELF_SLOTS] = {}; // 10 shelf towers (5 right, 5 front)
glm::vec3 shelfTierPos[10][SHELF_TIERS][SHELF_SLOTS]; // world pos of each slot

// Arm pick state machine
struct ShelfArm {
    glm::vec3 basePos;
    float baseRotY;
    float shoulderAngle = -30.0f;
    float targetAngle = -30.0f;
    int   pickBoxIndex = -1;  // index into gridBoxes being carried
    bool  carrying = false;
    float phase = 0.0f; // 0=idle/reaching, 1=lifted, 2=placing
    float phaseTimer = 0.0f;
    bool  armBusy = false;
    glm::vec3 pickupPos;
    glm::vec3 placePos;
    int towerIndex;
    glm::vec3 effectorPos = glm::vec3(0.0f);
};
ShelfArm shelfArms[10];

// Basic custom perspective
glm::mat4 customPerspective(float fovRadians, float aspect, float zNear, float zFar) {
    glm::mat4 result(0.0f);
    float tanHalfFovy = tan(fovRadians / 2.0f);

    result[0][0] = 1.0f / (aspect * tanHalfFovy);
    result[1][1] = 1.0f / (tanHalfFovy);
    result[2][2] = -(zFar + zNear) / (zFar - zNear);
    result[2][3] = -1.0f;
    result[3][2] = -(2.0f * zFar * zNear) / (zFar - zNear);

    return result;
}

// Binding arm: full body FIXED, only gripper fingers open/close
void drawBindingArm(Shader& shader, glm::vec3 basePos, float baseRotY, unsigned int darkTex, unsigned int lightTex, float gripperSpread) {
    // Fixed base pedestal
    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 base = glm::translate(glm::mat4(1.0f), basePos);
    base = glm::rotate(base, glm::radians(baseRotY), glm::vec3(0, 1, 0));
    shader.setMat4("model", glm::scale(base, glm::vec3(2.0f, 0.6f, 2.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 column = glm::translate(base, glm::vec3(0.0f, 0.6f, 0.0f));
    shader.setMat4("model", glm::scale(column, glm::vec3(0.8f, 2.4f, 0.8f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 elbow = glm::translate(column, glm::vec3(0.0f, 2.4f, 0.0f));
    shader.setMat4("model", glm::scale(elbow, glm::vec3(1.0f, 0.5f, 1.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Arm extends horizontally toward belt (in -Z direction)
    glm::mat4 reach = glm::translate(elbow, glm::vec3(0.0f, 0.5f, -1.0f));
    shader.setMat4("model", glm::scale(reach, glm::vec3(0.5f, 0.5f, 2.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Wrist plate at end of horizontal reach
    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 wrist = glm::translate(elbow, glm::vec3(0.0f, 0.5f, -2.1f));
    shader.setMat4("model", glm::scale(wrist, glm::vec3(0.8f, 0.3f, 0.3f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // GRIPPER FINGERS — only these move (spread opens/closes)
    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 f1 = glm::translate(wrist, glm::vec3(-gripperSpread, -0.4f, -0.15f));
    shader.setMat4("model", glm::scale(f1, glm::vec3(0.15f, 0.8f, 0.2f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glm::mat4 f2 = glm::translate(wrist, glm::vec3(gripperSpread, -0.4f, -0.15f));
    shader.setMat4("model", glm::scale(f2, glm::vec3(0.15f, 0.8f, 0.2f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

// ★ DRAW CEILING LIGHT FIXTURES ★
void drawCeilingLights(Shader& shader, unsigned int VAO, unsigned int lightTex, unsigned int yellowTex) {
    glBindVertexArray(VAO);
    
    // ★ FLUORESCENT TUBE LIGHTS - running along the X axis at Y=14.8 ★
    for (float x = -40.0f; x <= 40.0f; x += 15.0f) {
        // Main tube body (bright yellow to show emissive glow)
        glBindTexture(GL_TEXTURE_2D, yellowTex);
        glm::mat4 tubeModel = glm::translate(glm::mat4(1.0f), glm::vec3(x, 14.8f, 0.0f));
        tubeModel = glm::scale(tubeModel, glm::vec3(6.0f, 0.15f, 0.15f)); // Long tube
        shader.setMat4("model", tubeModel);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        
        // Mounting bracket (metal)
        glBindTexture(GL_TEXTURE_2D, lightTex);
        glm::mat4 bracketModel = glm::translate(glm::mat4(1.0f), glm::vec3(x, 14.9f, 0.0f));
        bracketModel = glm::scale(bracketModel, glm::vec3(0.3f, 0.2f, 0.3f));
        shader.setMat4("model", bracketModel);
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    
    // ★ INDUSTRIAL BULBS (POINT LIGHT SOURCES) ★
    // Large main bulb at center ceiling
    glBindTexture(GL_TEXTURE_2D, yellowTex);
    glm::mat4 mainBulb = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 14.7f, 0.0f));
    mainBulb = glm::scale(mainBulb, glm::vec3(0.4f, 0.4f, 0.4f)); // Sphere-like bulb
    shader.setMat4("model", mainBulb);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    
    // Mounting wire/stem
    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 stemModel = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 14.5f, 0.0f));
    stemModel = glm::scale(stemModel, glm::vec3(0.08f, 0.3f, 0.08f));
    shader.setMat4("model", stemModel);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    
    // ★ SIDE FACTORY LIGHTS (POINT LIGHTS 0 & 1) ★
    for (float x : {-10.0f, 10.0f}) {
        glBindTexture(GL_TEXTURE_2D, yellowTex);
        glm::mat4 sideBulb = glm::translate(glm::mat4(1.0f), glm::vec3(x, 14.7f, 0.0f));
        sideBulb = glm::scale(sideBulb, glm::vec3(0.35f, 0.35f, 0.35f));
        shader.setMat4("model", sideBulb);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        
        glBindTexture(GL_TEXTURE_2D, lightTex);
        glm::mat4 sideStem = glm::translate(glm::mat4(1.0f), glm::vec3(x, 14.5f, 0.0f));
        sideStem = glm::scale(sideStem, glm::vec3(0.06f, 0.3f, 0.06f));
        shader.setMat4("model", sideStem);
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
}

// Shelf arm: Industrial Stacker Crane
void drawShelfArm(Shader& shader, glm::vec3 basePos, glm::vec3 effectorPos, unsigned int darkTex, unsigned int lightTex) {
    // 1. Fixed tall Mast (from Y=0 up to Y=18)
    glBindTexture(GL_TEXTURE_2D, darkTex);
    glm::mat4 mast = glm::translate(glm::mat4(1.0f), glm::vec3(basePos.x, 9.0f, basePos.z));
    shader.setMat4("model", glm::scale(mast, glm::vec3(1.0f, 18.0f, 1.0f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // 2. Carriage (slides up/down the mast to match effectorPos.y)
    float boomY = effectorPos.y + 1.2f;

    glBindTexture(GL_TEXTURE_2D, lightTex);
    glm::mat4 carriage = glm::translate(glm::mat4(1.0f), glm::vec3(basePos.x, boomY, basePos.z));
    shader.setMat4("model", glm::scale(carriage, glm::vec3(1.5f, 1.5f, 1.5f)));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // 3. Boom (horizontal beam connecting mast to effector position)
    glm::vec2 boomStart(basePos.x, basePos.z);
    glm::vec2 boomEnd(effectorPos.x, effectorPos.z);
    glm::vec2 boomDir = boomEnd - boomStart;
    float boomLen = glm::length(boomDir);

    if (boomLen > 0.05f) {
        glm::vec2 dirNorm = boomDir / boomLen;
        float angleY = atan2(dirNorm.x, dirNorm.y);

        // Boom spans from carriage to the effector directly
        glm::vec3 boomCenter = glm::vec3(basePos.x + boomDir.x * 0.5f, boomY, basePos.z + boomDir.y * 0.5f);

        glBindTexture(GL_TEXTURE_2D, darkTex);
        glm::mat4 boom = glm::translate(glm::mat4(1.0f), boomCenter);
        boom = glm::rotate(boom, angleY, glm::vec3(0, 1, 0));
        shader.setMat4("model", glm::scale(boom, glm::vec3(0.5f, 0.5f, boomLen)));
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // 4. Gripper Head (at the end of the boom, hovering above the box)
        glBindTexture(GL_TEXTURE_2D, lightTex);
        glm::mat4 head = glm::translate(glm::mat4(1.0f), glm::vec3(effectorPos.x, boomY, effectorPos.z));
        shader.setMat4("model", glm::scale(head, glm::vec3(1.0f, 0.6f, 1.0f)));
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Gripper Claws mapping down to the box
        glBindTexture(GL_TEXTURE_2D, darkTex);
        for (float lx : {-0.4f, 0.4f}) {
            glm::mat4 claw = glm::translate(glm::mat4(1.0f), glm::vec3(effectorPos.x + lx * dirNorm.y, boomY - 0.6f, effectorPos.z - lx * dirNorm.x));
            shader.setMat4("model", glm::scale(claw, glm::vec3(0.1f, 1.2f, 0.8f))); // approximate bounding bracket
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }
}

unsigned int archVAO = 0;
int archVertexCount = 0;
void buildArch() {
    std::vector<float> verts;
    int segments = 20;
    float radius = 1.0f;
    float length = 1.0f;
    for (int i = 0; i <= segments; i++) {
        float theta = glm::pi<float>() * (float)i / (float)segments;
        float x = radius * cos(theta);
        float y = radius * sin(theta);
        float nx = cos(theta);
        float ny = sin(theta);
        float u = (float)i / segments;
        verts.push_back(x); verts.push_back(y); verts.push_back(length / 2);
        verts.push_back(nx); verts.push_back(ny); verts.push_back(0);
        verts.push_back(u); verts.push_back(1.0f);
        verts.push_back(x); verts.push_back(y); verts.push_back(-length / 2);
        verts.push_back(nx); verts.push_back(ny); verts.push_back(0);
        verts.push_back(u); verts.push_back(0.0f);
    }
    archVertexCount = (segments + 1) * 2;
    unsigned int VBO;
    glGenVertexArrays(1, &archVAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(archVAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
}

// Industrial Black Box Paint Chamber -> Arched Tunnel
void drawPaintChamber(Shader& shader, unsigned int cubeVAO, glm::vec3 pos, int lineIndex, unsigned int tunnelTex, unsigned int glowTex, unsigned int pipeTex) {
    bool isHorizontal = lines[lineIndex].isHorizontal;
    // New arched curvy tunnel
    glBindVertexArray(archVAO);
    glBindTexture(GL_TEXTURE_2D, tunnelTex);

    // Position perfectly gripping the outer rails of the belt
    glm::mat4 tunnel = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0.0f, 0.1f, 0.0f));
    if (isHorizontal) {
        tunnel = glm::rotate(tunnel, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    }
    // Scale: Radius=1.5f (width 3.0f to cover 2.9f rails), Length=6.0f
    shader.setMat4("model", glm::scale(tunnel, glm::vec3(1.5f, 1.6f, 6.0f)));
    glDrawArrays(GL_TRIANGLE_STRIP, 0, archVertexCount);

    glBindVertexArray(cubeVAO);

    // Glow pad bottom
    glBindTexture(GL_TEXTURE_2D, glowTex);
    glm::mat4 glow = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0.0f, 0.2f, 0.0f));
    glm::vec3 glowScale = isHorizontal ? glm::vec3(5.8f, 0.4f, 2.0f) : glm::vec3(2.0f, 0.4f, 5.8f);
    shader.setMat4("model", glm::scale(glow, glowScale));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Dynamic Sliding Doors
    glBindTexture(GL_TEXTURE_2D, pipeTex);
    float len = glm::length(lines[lineIndex].end - lines[lineIndex].start);
    float midDist = len * 0.5f;

    for (float zOffset : {-3.0f, 3.0f}) {
        float doorDist = midDist + zOffset;
        float min_dist = 999.0f;
        for (const GridBox& b : gridBoxes) {
            if (b.state == ON_BELT && b.lineIndex == lineIndex) {
                float dist = std::abs(b.distance - doorDist);
                if (dist < min_dist) min_dist = dist;
            }
        }

        // Open the door totally if box is closer than 1.0, start opening at 2.2
        float openRatio = 0.0f;
        if (min_dist < 2.2f) {
            openRatio = glm::clamp((2.2f - min_dist) / 1.2f, 0.0f, 1.0f);
        }

        glm::vec3 curPos = pos + glm::vec3(0.0f, 0.9f, 0.0f);
        if (isHorizontal) curPos.x += zOffset; else curPos.z += zOffset;

        // 1. Draw Door Housings (Pillars)
        glBindTexture(GL_TEXTURE_2D, pipeTex);
        for (float sideSign : {-1.0f, 1.0f}) {
            glm::vec3 pillarPos = curPos;
            if (isHorizontal) pillarPos.z += sideSign * 1.05f;
            else pillarPos.x += sideSign * 1.05f;

            glm::mat4 pillar = glm::translate(glm::mat4(1.0f), pillarPos);
            glm::vec3 pScale = isHorizontal ? glm::vec3(0.12f, 1.6f, 0.8f) : glm::vec3(0.8f, 1.6f, 0.12f);
            shader.setMat4("model", glm::scale(pillar, pScale));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        // 2. Draw Top Header to connect pillars
        {
            glm::vec3 headerPos = curPos;
            headerPos.y += 0.65f;
            glm::mat4 header = glm::translate(glm::mat4(1.0f), headerPos);
            glm::vec3 hScale = isHorizontal ? glm::vec3(0.12f, 0.3f, 2.9f) : glm::vec3(2.9f, 0.3f, 0.12f);
            shader.setMat4("model", glm::scale(header, hScale));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        // 3. Draw sliding doors behind the housings
        glBindTexture(GL_TEXTURE_2D, pipeTex);
        for (float sideSign : {-1.0f, 1.0f}) {
            glm::vec3 doorPos = curPos;
            if (isHorizontal) doorPos.x -= (zOffset > 0 ? 0.05f : -0.05f); // hide into the tunnel bounds
            else doorPos.z -= (zOffset > 0 ? 0.05f : -0.05f);

            float closeOffset = 0.33f;
            float slideDist = openRatio * 0.65f;

            if (isHorizontal) doorPos.z += sideSign * (closeOffset + slideDist);
            else doorPos.x += sideSign * (closeOffset + slideDist);

            glm::mat4 doorMat = glm::translate(glm::mat4(1.0f), doorPos);
            glm::vec3 dScale = isHorizontal ? glm::vec3(0.04f, 1.3f, 0.65f) : glm::vec3(0.65f, 1.3f, 0.04f);

            shader.setMat4("model", glm::scale(doorMat, dScale));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }
}

// Helper function to find the project root directory and construct resource paths
std::string getResourcePath(const char* resourceType, const char* resourceName) {
    // Get the current working directory
    char cwd[256];
    _getcwd(cwd, sizeof(cwd));
    std::string currentPath(cwd);

    std::cout << "DEBUG: Searching for " << resourceType << "/" << resourceName
        << " starting from: " << currentPath << std::endl;

    // Try to find the resource directory by searching up to 10 levels
    for (int i = 0; i < 10; ++i) {
        std::string resourceDirName = std::string(resourceType) + "s";  // "shaders" or "textures"
        std::string resourcePath = currentPath + "\\" + resourceDirName + "\\" + resourceName;

        // Check if file exists
        FILE* file = nullptr;
        fopen_s(&file, resourcePath.c_str(), "rb");
        if (file) {
            fclose(file);
            std::cout << "✓ Found: " << resourcePath << std::endl;
            return resourcePath;
        }

        std::cout << "  Checked (not found): " << resourcePath << std::endl;

        // Also try single folder (consolidated) structure
        std::string singleFolderPath = currentPath + "\\" + resourceName;
        fopen_s(&file, singleFolderPath.c_str(), "rb");
        if (file) {
            fclose(file);
            std::cout << "✓ Found in consolidated folder: " << singleFolderPath << std::endl;
            return singleFolderPath;
        }

        // Go up one directory
        size_t lastSlash = currentPath.find_last_of("\\");
        if (lastSlash == std::string::npos) break;
        currentPath = currentPath.substr(0, lastSlash);
    }

    // If not found, print debug info and return fallback
    std::cout << "✗ ERROR: Could not find resource '" << resourceName << "'" << std::endl;
    std::cout << "  Current working directory: " << cwd << std::endl;
    std::cout << "  SOLUTION: Set Working Directory in Project Properties to where your resources are located" << std::endl;
    std::cout << "  Example: D:\\4-2\\graProject\\FactoryFlow3D\\" << std::endl;

    // Fallback: return the original request
    std::string fallback = std::string(resourceType) + "s/";
    fallback += resourceName;
    return fallback;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow* window);
unsigned int loadTexture(const char* path, unsigned char r = 150, unsigned char g = 150, unsigned char b = 150);
void renderScene(Shader& shader, unsigned int VAO, unsigned int boxTex, unsigned int conveyorTex, unsigned int floorTex, unsigned int wallTex, unsigned int blueTex, unsigned int tunnelTex, unsigned int whiteLightTex, glm::mat4 view, glm::mat4 projection, glm::vec3 camPos);

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "FactoryFlow 3D", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    mainCamera.Mode = REALISTIC;
    birdEyeCamera.Mode = BIRD_EYE;
    followCamera.Mode = FOLLOW;
    frontCamera.Mode = ASSIGNMENT;
    frontCamera.Position = glm::vec3(0.0f, 7.0f, 15.0f);
    frontCamera.Pitch = -15.0f; // Look slightly down

    std::string phongVsPath = getResourcePath("shader", "vertexShaderForPhongShading.vs");
    std::string phongFsPath = getResourcePath("shader", "fragmentShaderForPhongShading.fs");
    std::string gouraudVsPath = getResourcePath("shader", "vertexShaderForGouraudShading.vs");
    std::string gouraudFsPath = getResourcePath("shader", "fragmentShaderForGouraudShading.fs");

    Shader phongShader(phongVsPath.c_str(), phongFsPath.c_str());
    Shader gouraudShader(gouraudVsPath.c_str(), gouraudFsPath.c_str());

    float vertices[] = {
        // positions          // normals           // texture coords
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,

        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f
    };

    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    buildArch();

    std::string boxTexPath = getResourcePath("texture", "box.jpg");
    std::string conveyorTexPath = getResourcePath("texture", "conveyor.jpg");
    std::string floorTexPath = getResourcePath("texture", "floor.jpg");
    std::string wallTexPath = getResourcePath("texture", "wall.jpg");
    std::string blueTexPath = getResourcePath("texture", "blue.jpg");
    std::string tunnelTexPath = getResourcePath("texture", "tunnel.png");

    unsigned int boxTexture = loadTexture(boxTexPath.c_str(), 160, 100, 50); // Wooden brown
    unsigned int conveyorTexture = loadTexture(conveyorTexPath.c_str(), 30, 30, 30); // Very Dark gray
    unsigned int floorTexture = loadTexture(floorTexPath.c_str(), 180, 180, 180); // Light gray
    unsigned int wallTexture = loadTexture(wallTexPath.c_str(), 120, 140, 160); // Slate blue/gray
    unsigned int blueTexture = loadTexture(blueTexPath.c_str(), 0, 150, 255); // Cyan-blue
    unsigned int tunnelTexture = loadTexture(tunnelTexPath.c_str(), 200, 200, 200); // Tunnel sci-fi plates
    unsigned int whiteLightTexture = loadTexture("", 255, 255, 255); // Bright white for light fixtures (no path = fallback)

    // Populate hundreds of boxes densely on all grid lines
    for (int i = 0; i < lines.size(); i++) {
        float len = glm::length(lines[i].end - lines[i].start);
        int numBoxes = (int)(len / 4.0f); // Space them exactly per line length
        float spacing = len / (float)numBoxes;
        for (int b = 0; b < numBoxes; b++) {
            float dist = b * spacing;
            BoxStage stage = RAW;
            if (dist > len * 0.5f) {
                stage = (i == 2) ? BOUND : PAINTED;
            }
            gridBoxes.push_back({ i, dist, stage });
        }
    }

    // Pre-compute shelf slot positions for the 10 destination shelves
    // 5 right shelves (X=46) for horizontal belts
    for (int tower = 0; tower < 5; tower++) {
        float shelfZ = -20.0f + tower * 10.0f;
        for (int tier = 0; tier < SHELF_TIERS; tier++) {
            for (int slot = 0; slot < SHELF_SLOTS; slot++) {
                float bz = -3.5f + slot * 1.0f; // spread across shelf depth
                shelfTierPos[tower][tier][slot] = glm::vec3(46.0f, tier * 3.0f + 0.6f + 0.5f, shelfZ + bz);
            }
        }
    }
    // 5 front shelves (Z=46) for vertical belts
    for (int tower = 0; tower < 5; tower++) {
        float shelfX = -20.0f + tower * 10.0f;
        for (int tier = 0; tier < SHELF_TIERS; tier++) {
            for (int slot = 0; slot < SHELF_SLOTS; slot++) {
                float bx = -3.5f + slot * 1.0f; // spread across shelf width
                shelfTierPos[5 + tower][tier][slot] = glm::vec3(shelfX + bx, tier * 3.0f + 0.6f + 0.5f, 46.0f);
            }
        }
    }

    // Initialize the 10 shelf arms
    for (int i = 0; i < 5; i++) {
        float z = -20.0f + i * 10.0f;
        shelfArms[i].basePos = glm::vec3(38.0f, 0.0f, z + 2.5f); // Right side arms
        shelfArms[i].baseRotY = -90.0f; // Face left towards belt
        shelfArms[i].towerIndex = i;

        float x = -20.0f + i * 10.0f;
        shelfArms[5 + i].basePos = glm::vec3(x + 2.5f, 0.0f, 38.0f); // Front side arms
        shelfArms[5 + i].baseRotY = 180.0f; // Face back towards belt
        shelfArms[5 + i].towerIndex = 5 + i;
    }

    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        // Update animations
        if (fanOn) fanAngle += 150.0f * deltaTime;
        robotBaseAngle += 20.0f * deltaTime;
        robotElbowAngle = sin(glfwGetTime() * 2.0f) * 30.0f;
        // Gripper cycles: open(0.7) -> close(0.1) -> hold -> repeat
        gripperSpread = 0.4f + 0.35f * sin(glfwGetTime() * 2.5f); // oscillates 0.05..0.75

        // --- BOX PROCESSING STAGE MACHINE ---
        float bindPositions[3] = { 30.0f, 45.0f, 60.0f }; // X = -10, 5, 20 on Z=0 belt
        float bindRadius = 4.0f;
        float conveyorSpeed = 1.5f;

        // ----- BOX STATE MACHINE -----
        std::vector<GridBox> newBoxes;

        for (int i = 0; i < (int)gridBoxes.size(); i++) {
            GridBox& b = gridBoxes[i];

            if (b.state == ON_BELT) {
                float lineLen = glm::length(lines[b.lineIndex].end - lines[b.lineIndex].start);

                // Unified paint zone perfectly matching the central Paint Chamber!
                // Box turns blue identically at the exact halfway point, hidden under the machine shell.
                if (b.distance >= lineLen * 0.5f && b.stage == RAW) {
                    b.stage = PAINTED;
                }

                // Horizontal belt line 2 (Z=0) has the binding stations along its path
                if (b.lineIndex == 2) {
                    for (int arm = 0; arm < 3; arm++) {
                        if (b.distance >= bindPositions[arm] - bindRadius &&
                            b.distance <= bindPositions[arm] + bindRadius)
                            b.stage = BOUND;
                    }
                }

                // --- Move along belt ---
                b.distance += conveyorSpeed * deltaTime;
                float exitDist = lineLen - 1.5f; // End of belt near shelf

                if (b.distance >= exitDist) {
                    if (b.stage == PAINTED || b.stage == BOUND) {
                        // COLORED box: wait at the end of the belt for robotic arm
                        glm::vec3 dir = glm::normalize(lines[b.lineIndex].end - lines[b.lineIndex].start);
                        b.worldPos = lines[b.lineIndex].start + dir * exitDist;
                        b.worldPos.y = lines[b.lineIndex].start.y + 0.6f;
                        b.state = WAITING_FOR_PICKUP;
                        // Spawn replacement box just inside start shelf so it rolls out smoothly
                        GridBox nb;
                        nb.lineIndex = b.lineIndex;
                        nb.distance = -4.0f;
                        nb.stage = RAW;
                        nb.state = ON_BELT;
                        newBoxes.push_back(nb);
                    }
                    else {
                        // RAW box never got colored - loops back to start
                        b.distance -= lineLen;
                        b.stage = RAW;
                    }
                }

            }
            else if (b.state == WAITING_FOR_PICKUP) {
                // Box sits quietly at the end of the horizontal belt
                // Robotic arm logic will find it and change state to BEING_PICKED

            }
            else if (b.state == BEING_PICKED) {
                // Picked by arm. Just safety timeout:
                b.pickTimer += deltaTime;
                if (b.pickTimer > 8.0f) {
                    int tIdx = b.lineIndex;
                    bool placed = false;
                    for (int t = 0; t < SHELF_TIERS && !placed; t++) {
                        for (int s = 0; s < SHELF_SLOTS && !placed; s++) {
                            if (!shelfOccupied[tIdx][t][s]) {
                                shelfOccupied[tIdx][t][s] = true;
                                b.worldPos = shelfTierPos[tIdx][t][s];
                                b.stage = BOUND;
                                b.state = ON_SHELF;
                                placed = true;
                            }
                        }
                    }
                    if (!placed) b.state = ON_SHELF;
                }

            }
            else if (b.state == ON_SHELF) {
                b.onShelfT += deltaTime;
            }
        }

        // Add newly spawned replacement boxes
        for (auto& nb : newBoxes)
            gridBoxes.push_back(nb);

        // Arm reach (binding arm oscillation)
        armReachAngle = -30.0f + sin(glfwGetTime() * 3.0f) * 28.0f;

        // ----- SHELF ARM AI -----
        for (int a = 0; a < 10; a++) {
            ShelfArm& arm = shelfArms[a];

            if (!arm.armBusy) {
                // Look for a box waiting at the end of THIS arm's belt
                int closest = -1;
                for (int i = 0; i < (int)gridBoxes.size(); i++) {
                    if (gridBoxes[i].state == WAITING_FOR_PICKUP && gridBoxes[i].lineIndex == arm.towerIndex) {
                        closest = i;
                        break;
                    }
                }
                if (closest >= 0) {
                    // Find a free shelf slot on this tower
                    for (int t = 0; t < SHELF_TIERS; t++) {
                        for (int s = 0; s < SHELF_SLOTS; s++) {
                            if (!shelfOccupied[arm.towerIndex][t][s]) {
                                shelfOccupied[arm.towerIndex][t][s] = true;
                                arm.pickBoxIndex = closest;
                                arm.pickupPos = gridBoxes[closest].worldPos;
                                arm.placePos = shelfTierPos[arm.towerIndex][t][s];
                                gridBoxes[closest].state = BEING_PICKED;
                                gridBoxes[closest].pickTimer = 0.0f;
                                arm.armBusy = true;
                                arm.phase = 0.0f;
                                arm.phaseTimer = 0.0f;
                                arm.carrying = false;
                                // Reset idle position precisely
                                arm.effectorPos = arm.basePos + glm::vec3(0, 10.0f, 0);
                                goto nextArm;
                            }
                        }
                    }
                }
            }
            else {
                arm.phaseTimer += deltaTime;
                glm::vec3 idlePos = arm.basePos + glm::vec3(0, 10.0f, 0);

                if (arm.phase == 0.0f) {
                    // Fast reach DOWN toward pickup (0.4s)
                    float t = glm::clamp(arm.phaseTimer / 0.4f, 0.0f, 1.0f);
                    t = t * t * (3.0f - 2.0f * t); // smoothstep
                    arm.effectorPos = glm::mix(idlePos, arm.pickupPos, t);

                    if (arm.phaseTimer >= 0.4f) {
                        arm.phase = 1.0f;
                        arm.phaseTimer = 0.0f;
                        arm.carrying = true; // Box grabbed!
                    }
                }
                else if (arm.phase == 1.0f) {
                    // Fast Lift UP with the box (0.3s)
                    float t = glm::clamp(arm.phaseTimer / 0.3f, 0.0f, 1.0f);
                    t = t * t * (3.0f - 2.0f * t);
                    glm::vec3 liftPos = arm.pickupPos + glm::vec3(0, 4.0f, 0);
                    arm.effectorPos = glm::mix(arm.pickupPos, liftPos, t);

                    if (arm.phaseTimer >= 0.3f) { arm.phase = 2.0f; arm.phaseTimer = 0.0f; }
                }
                else if (arm.phase == 2.0f) {
                    // Fast swing toward shelf & lower (0.6s)
                    float t = glm::clamp(arm.phaseTimer / 0.6f, 0.0f, 1.0f);
                    t = t * t * (3.0f - 2.0f * t);

                    glm::vec3 liftPos = arm.pickupPos + glm::vec3(0, 4.0f, 0);
                    glm::vec3 dropPos = arm.placePos + glm::vec3(0, 4.0f, 0);

                    if (t < 0.7f) {
                        arm.effectorPos = glm::mix(liftPos, dropPos, t / 0.7f);
                    }
                    else {
                        arm.effectorPos = glm::mix(dropPos, arm.placePos, (t - 0.7f) / 0.3f);
                    }

                    if (arm.phaseTimer >= 0.6f) {
                        // Place box!
                        if (arm.pickBoxIndex >= 0 && arm.pickBoxIndex < (int)gridBoxes.size()) {
                            gridBoxes[arm.pickBoxIndex].worldPos = arm.placePos;
                            gridBoxes[arm.pickBoxIndex].state = ON_SHELF;
                            gridBoxes[arm.pickBoxIndex].stage = BOUND;
                        }
                        arm.armBusy = false;
                        arm.carrying = false;
                        arm.pickBoxIndex = -1;
                        arm.phase = 0.0f;
                        arm.phaseTimer = 0.0f;
                    }
                }

                // MATHEMATICAL GLUE: Box perfectly tracks the crane effector
                if (arm.carrying && arm.pickBoxIndex >= 0 && arm.pickBoxIndex < (int)gridBoxes.size()) {
                    gridBoxes[arm.pickBoxIndex].worldPos = arm.effectorPos;
                }
            }

            // Auto return to idle when not busy
            if (!arm.armBusy) {
                arm.effectorPos = glm::mix(arm.effectorPos, arm.basePos + glm::vec3(0, 10.0f, 0), deltaTime * 2.0f);
            }
        nextArm:;
        }

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Follow a random active box
        if (!gridBoxes.empty()) {
            glm::vec3 dir = glm::normalize(lines[0].end - lines[0].start);
            glm::vec3 tgtPos = lines[0].start + dir * gridBoxes[0].distance;
            followCamera.SetTarget(tgtPos);
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        float aspect = singleViewport ? (float)width / (float)height : (float)(width / 2) / (float)(height / 2);
        glm::mat4 projection = customPerspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

        if (singleViewport) {
            glViewport(0, 0, width, height);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, whiteLightTexture, mainCamera.GetViewMatrix(), projection, mainCamera.Position);
        }
        else {
            // 1. Top-Left Viewport (Main Interactive Camera)
            glViewport(0, height / 2, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, whiteLightTexture, mainCamera.GetViewMatrix(), projection, mainCamera.Position);

            // 2. Top-Right Viewport (Bird's Eye)
            glViewport(width / 2, height / 2, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, whiteLightTexture, birdEyeCamera.GetViewMatrix(), projection, birdEyeCamera.Position);

            // 3. Bottom-Left Viewport (Follow Camera)
            glViewport(0, 0, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, whiteLightTexture, followCamera.GetViewMatrix(), projection, followCamera.Position);

            // 4. Bottom-Right Viewport (Static Front Camera)
            glViewport(width / 2, 0, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, whiteLightTexture, frontCamera.GetViewMatrix(), projection, frontCamera.Position);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glfwTerminate();
    return 0;
}

void renderScene(Shader& shader, unsigned int VAO, unsigned int boxTex, unsigned int conveyorTex, unsigned int floorTex, unsigned int wallTex, unsigned int blueTex, unsigned int tunnelTex, unsigned int whiteLightTex, glm::mat4 view, glm::mat4 projection, glm::vec3 camPos) {
    shader.use();
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);

    // Pass texture fallback info
    shader.setBool("useTextureColorOnly", useTextureColorOnly);

    // Explicitly set material uniforms
    shader.setInt("material.diffuse", 0);
    shader.setInt("material.specular", 0); // use diffuse map for specular as well for simplicity
    shader.setFloat("material.shininess", 32.0f);

    shader.setBool("ambientOn", masterLightOn && ambientOn);
    shader.setBool("diffuseOn", masterLightOn && diffuseOn);
    shader.setBool("specularOn", masterLightOn && specularOn);
    shader.setBool("dirLightOn", masterLightOn && dirLightOn);
    shader.setBool("pointLightOn", masterLightOn && pointLightOn);
    shader.setBool("spotLightOn", masterLightOn && spotLightOn);

    // Setup Directional Light
    shader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
    shader.setVec3("dirLight.ambient", 0.3f, 0.3f, 0.3f);
    shader.setVec3("dirLight.diffuse", 0.5f, 0.5f, 0.5f);
    shader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);

    // Setup Point Lights (1 green, 1 red for factory lighting)
    shader.setVec3("pointLights[0].position", -10.0f, 8.0f, 0.0f);
    shader.setVec3("pointLights[0].ambient", 0.0f, 0.1f, 0.3f);
    shader.setVec3("pointLights[0].diffuse", 0.3f, 0.6f, 1.0f); // Industrial bluish
    shader.setVec3("pointLights[0].specular", 1.0f, 1.0f, 1.0f);
    shader.setFloat("pointLights[0].constant", 1.0f);
    shader.setFloat("pointLights[0].linear", 0.045f);
    shader.setFloat("pointLights[0].quadratic", 0.0075f);

    shader.setVec3("pointLights[1].position", 10.0f, 8.0f, 0.0f);
    shader.setVec3("pointLights[1].ambient", 0.0f, 0.1f, 0.3f);
    shader.setVec3("pointLights[1].diffuse", 0.3f, 0.6f, 1.0f);
    shader.setVec3("pointLights[1].specular", 1.0f, 1.0f, 1.0f);
    shader.setFloat("pointLights[1].constant", 1.0f);
    shader.setFloat("pointLights[1].linear", 0.045f);
    shader.setFloat("pointLights[1].quadratic", 0.0075f);

    // Setup Main Roof Light (pointLights[2])
    shader.setVec3("pointLights[2].position", 0.0f, 9.8f, 0.0f);
    if (mainLightOn && masterLightOn && pointLightOn) {
        shader.setVec3("pointLights[2].ambient", 0.2f, 0.2f, 0.2f);
        shader.setVec3("pointLights[2].diffuse", 1.0f, 1.0f, 1.0f);
        shader.setVec3("pointLights[2].specular", 1.0f, 1.0f, 1.0f);
    }
    else {
        shader.setVec3("pointLights[2].ambient", 0.0f, 0.0f, 0.0f);
        shader.setVec3("pointLights[2].diffuse", 0.0f, 0.0f, 0.0f);
        shader.setVec3("pointLights[2].specular", 0.0f, 0.0f, 0.0f);
    }
    shader.setFloat("pointLights[2].constant", 1.0f);
    shader.setFloat("pointLights[2].linear", 0.045f);
    shader.setFloat("pointLights[2].quadratic", 0.0075f);

    // Disable pointLight 3 to avoid artifacting
    shader.setVec3("pointLights[3].ambient", 0.0f, 0.0f, 0.0f);
    shader.setVec3("pointLights[3].diffuse", 0.0f, 0.0f, 0.0f);
    shader.setVec3("pointLights[3].specular", 0.0f, 0.0f, 0.0f);
    shader.setFloat("pointLights[3].constant", 1.0f);
    shader.setFloat("pointLights[3].linear", 0.09f);
    shader.setFloat("pointLights[3].quadratic", 0.032f);

    // Setup Spot Light (Camera flashlight)
    shader.setVec3("spotLight.position", mainCamera.Position);
    shader.setVec3("spotLight.direction", mainCamera.Front);
    shader.setVec3("spotLight.ambient", 0.0f, 0.0f, 0.0f);
    shader.setVec3("spotLight.diffuse", 1.0f, 1.0f, 1.0f);
    shader.setVec3("spotLight.specular", 1.0f, 1.0f, 1.0f);
    shader.setFloat("spotLight.constant", 1.0f);
    shader.setFloat("spotLight.linear", 0.09f);
    shader.setFloat("spotLight.quadratic", 0.032f);
    shader.setFloat("spotLight.cutOff", glm::cos(glm::radians(12.5f)));
    shader.setFloat("spotLight.outerCutOff", glm::cos(glm::radians(15.0f)));

    shader.setVec3("viewPos", camPos);

    glBindVertexArray(VAO);
    glActiveTexture(GL_TEXTURE0);

    // 1. DRAW SPRAWLING CONVEYOR GRID NETWORK
    for (size_t i = 0; i < lines.size(); i++) {
        glm::vec3 dir = lines[i].end - lines[i].start;
        float len = glm::length(dir);
        glm::vec3 center = lines[i].start + dir * 0.5f;
        float angle = atan2(dir.x, dir.z);

        glm::mat4 model = glm::translate(glm::mat4(1.0f), center);
        model = glm::rotate(model, angle, glm::vec3(0, 1, 0));

        // Main flat belt track
        glBindTexture(GL_TEXTURE_2D, conveyorTex);
        glm::mat4 trackModel = glm::scale(model, glm::vec3(2.8f, 0.2f, len)); // belt
        shader.setMat4("model", trackModel);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Rounded roller "drums" at start and end of track
        glBindTexture(GL_TEXTURE_2D, wallTex);
        glm::mat4 drumStart = glm::translate(model, glm::vec3(0.0f, -0.05f, -len / 2.0f));
        shader.setMat4("model", glm::scale(drumStart, glm::vec3(2.7f, 0.5f, 0.5f)));
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glm::mat4 drumEnd = glm::translate(model, glm::vec3(0.0f, -0.05f, len / 2.0f));
        shader.setMat4("model", glm::scale(drumEnd, glm::vec3(2.7f, 0.5f, 0.5f)));
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Glowing Blue Guide Rails
        glBindTexture(GL_TEXTURE_2D, blueTex);
        glm::mat4 rail1 = glm::translate(model, glm::vec3(-1.45f, 0.15f, 0.0f));
        shader.setMat4("model", glm::scale(rail1, glm::vec3(0.1f, 0.3f, len)));
        glDrawArrays(GL_TRIANGLES, 0, 36);

        glm::mat4 rail2 = glm::translate(model, glm::vec3(1.45f, 0.15f, 0.0f));
        shader.setMat4("model", glm::scale(rail2, glm::vec3(0.1f, 0.3f, len)));
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Concrete support legs repeating along track length
        glBindTexture(GL_TEXTURE_2D, wallTex);
        int numLegs = (int)(len / 4.0f);
        float legLength = lines[i].start.y + 0.9f; // dynamically reach the floor at -0.7f
        float legYOffset = -legLength / 2.0f;
        for (int j = 0; j <= numLegs; j++) {
            float dist = j * 4.0f - len / 2.0f;
            glm::mat4 legCenter = glm::translate(model, glm::vec3(0.0f, 0.0f, dist));

            glm::mat4 legL = glm::translate(legCenter, glm::vec3(-1.3f, legYOffset, 0.0f));
            shader.setMat4("model", glm::scale(legL, glm::vec3(0.2f, legLength, 0.2f)));
            glDrawArrays(GL_TRIANGLES, 0, 36);

            glm::mat4 legR = glm::translate(legCenter, glm::vec3(1.3f, legYOffset, 0.0f));
            shader.setMat4("model", glm::scale(legR, glm::vec3(0.2f, legLength, 0.2f)));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // 2. DRAW MASSIVE BOX POPULATION (Animated)
    for (size_t i = 0; i < gridBoxes.size(); i++) {
        const GridBox& gb = gridBoxes[i];
        glm::vec3 pos;
        float angle = 0.0f;

        if (gb.state == ON_BELT) {
            glm::vec3 dir = glm::normalize(lines[gb.lineIndex].end - lines[gb.lineIndex].start);
            pos = lines[gb.lineIndex].start + dir * gb.distance;
            pos.y = lines[gb.lineIndex].start.y + 0.6f;
            angle = atan2(dir.x, dir.z);
        }
        else {
            // Off-belt states: use worldPos
            pos = gb.worldPos;
        }

        // Choose color/texture by stage
        if (gb.stage == PAINTED || gb.stage == BOUND) {
            glBindTexture(GL_TEXTURE_2D, blueTex);
        }
        else {
            glBindTexture(GL_TEXTURE_2D, boxTex);
        }

        glm::mat4 model = glm::translate(glm::mat4(1.0f), pos);
        model = glm::rotate(model, angle, glm::vec3(0, 1, 0));
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));
        shader.setMat4("model", model);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // If BOUND: draw rope bands over the box
        if (gb.stage == BOUND) {
            glBindTexture(GL_TEXTURE_2D, wallTex);
            float offsets[3] = { -0.3f, 0.0f, 0.3f };
            for (int r = 0; r < 3; r++) {
                glm::mat4 rope = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0.0f, offsets[r], 0.0f));
                rope = glm::rotate(rope, angle, glm::vec3(0, 1, 0));
                rope = glm::scale(rope, glm::vec3(1.05f, 0.12f, 1.05f));
                shader.setMat4("model", rope);
                glDrawArrays(GL_TRIANGLES, 0, 36);
            }
        }
    }

    // 3. DRAW MASSIVE SHELVING — Left/Right Walls & Back/Front Walls
    for (int wall = 0; wall < 4; wall++) {
        // wall 0: Left (X=-46), wall 1: Right (X=46)
        // wall 2: Back (Z=-46), wall 3: Front (Z=46)
        bool isBoundSide = (wall == 1 || wall == 3);
        float mainCoord = (wall == 0 || wall == 2) ? -46.0f : 46.0f;
        bool isHorizontal = (wall < 2);

        for (int tower = 0; tower < 5; tower++) {
            float crossOffset = -20.0f + tower * 10.0f;
            float shelfX = isHorizontal ? mainCoord : crossOffset;
            float shelfZ = isHorizontal ? crossOffset : mainCoord;

            // Horizontal shelf tiers
            for (int y = 0; y < 5; y++) {
                glBindTexture(GL_TEXTURE_2D, conveyorTex);
                glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX, y * 3.0f, shelfZ));
                model = glm::scale(model, isHorizontal ? glm::vec3(4.0f, 0.2f, 9.0f) : glm::vec3(9.0f, 0.2f, 4.0f));
                shader.setMat4("model", model);
                glDrawArrays(GL_TRIANGLES, 0, 36);

                // Aesthetic static boxes to make shelves look full initially
                for (float b_cross = -3.5f; b_cross <= 3.5f; b_cross += 1.5f) {
                    if ((int)(b_cross * 10.0f + y + tower) % 4 == 0) continue; // natural gaps

                    // Only draw static boxes on the RAW sides
                    if (!isBoundSide) {
                        glBindTexture(GL_TEXTURE_2D, boxTex);
                        float bx = isHorizontal ? 0.0f : b_cross;
                        float bz = isHorizontal ? b_cross : 0.0f;
                        glm::mat4 bModel = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX + bx, y * 3.0f + 0.6f, shelfZ + bz));
                        shader.setMat4("model", glm::scale(bModel, glm::vec3(1.2f, 1.0f, 1.0f)));
                        glDrawArrays(GL_TRIANGLES, 0, 36);
                    }
                }
            }

            // Vertical posts for the tower
            glBindTexture(GL_TEXTURE_2D, conveyorTex);
            float p_main = 1.8f, p_cross = 4.3f;
            float pxArr[2] = { isHorizontal ? -p_main : -p_cross, isHorizontal ? p_main : p_cross };
            float pzArr[2] = { isHorizontal ? -p_cross : -p_main, isHorizontal ? p_cross : p_main };

            for (int p_i = 0; p_i < 2; p_i++) {
                for (int p_j = 0; p_j < 2; p_j++) {
                    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(shelfX + pxArr[p_i], 6.0f, shelfZ + pzArr[p_j]));
                    model = glm::scale(model, glm::vec3(0.4f, 12.0f, 0.4f));
                    shader.setMat4("model", model);
                    glDrawArrays(GL_TRIANGLES, 0, 36);
                }
            }
        }
    }

    // 3.5 PAINT CHAMBERS — on EVERY BELT
    for (size_t i = 0; i < lines.size(); i++) {
        glm::vec3 midPoint = (lines[i].start + lines[i].end) * 0.5f;
        drawPaintChamber(shader, VAO, midPoint, i, tunnelTex, blueTex, wallTex);
    }

    // 4a. BINDING ARM — 1 arm beside lower belt at Z=0 (near binding zone at X=-10)
    //     Body FIXED, only gripper opens/closes around the passing box
    drawBindingArm(shader, glm::vec3(-10.0f, 0.0f, 3.0f), 180.0f, conveyorTex, wallTex, gripperSpread);

    // 4b. SHELF PLACEMENT ARMS — 10 Cartesian Stacker Cranes
    for (int i = 0; i < 10; i++) {
        drawShelfArm(shader, shelfArms[i].basePos, shelfArms[i].effectorPos, conveyorTex, wallTex);
    }

    // 5. DRAW MASSIVE WAREHOUSE ROOM (Floor, Concrete Pillars, Ceiling)
    glBindTexture(GL_TEXTURE_2D, floorTex);

    // Concrete Floor (100x100)
    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.7f, 0.0f));
    model = glm::scale(model, glm::vec3(100.0f, 0.1f, 100.0f));
    shader.setMat4("model", model);
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Ceiling Base
    model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 15.0f, 0.0f));
    model = glm::scale(model, glm::vec3(100.0f, 0.1f, 100.0f));
    shader.setMat4("model", model);
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // ★ DRAW CEILING LIGHT FIXTURES ★
    drawCeilingLights(shader, VAO, wallTex, whiteLightTex);

    // Large Main Concrete Pillars
    float pilX[6] = { -25.0f, 25.0f, -25.0f, 25.0f, 0.0f, 0.0f };
    float pilZ[6] = { -25.0f, -25.0f, 25.0f, 25.0f, -25.0f, 25.0f };
    for (int i = 0; i < 6; i++) {
        model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(pilX[i], 7.0f, pilZ[i]));
        model = glm::scale(model, glm::vec3(2.5f, 16.0f, 2.5f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }

    // Warehouse Walls Boundaries
    glBindTexture(GL_TEXTURE_2D, wallTex);
    // Back Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 7.0f, -50.0f));
    model = glm::scale(model, glm::vec3(100.0f, 16.0f, 0.1f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    // Front Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 7.0f, 50.0f));
    model = glm::scale(model, glm::vec3(100.0f, 16.0f, 0.1f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    // Left Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(-50.0f, 7.0f, 0.0f));
    model = glm::scale(model, glm::vec3(0.1f, 16.0f, 100.0f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    // Right Wall
    model = glm::translate(glm::mat4(1.0f), glm::vec3(50.0f, 7.0f, 0.0f));
    model = glm::scale(model, glm::vec3(0.1f, 16.0f, 100.0f));
    shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (mainCamera.Mode == REALISTIC || mainCamera.Mode == ASSIGNMENT) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) mainCamera.ProcessKeyboard(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) mainCamera.ProcessKeyboard(BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) mainCamera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) mainCamera.ProcessKeyboard(RIGHT, deltaTime);
    }

    // Assignment mode specific controls
    if (mainCamera.Mode == ASSIGNMENT) {
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) mainCamera.ProcessKeyboard(UP, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) mainCamera.ProcessKeyboard(DOWN, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) mainCamera.ProcessKeyboard(PITCH_UP, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS) mainCamera.ProcessKeyboard(YAW_LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) mainCamera.ProcessKeyboard(ROLL_LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) mainCamera.ProcessKeyboard(ROTATE_AROUND, deltaTime);
    }

    // Toggles logic
#define DO_TOGGLE(KEY, STATE_VAR, PRESSED_VAR) \
        if (glfwGetKey(window, KEY) == GLFW_PRESS) { \
            if (!PRESSED_VAR) { STATE_VAR = !STATE_VAR; PRESSED_VAR = true; } \
        } else { PRESSED_VAR = false; }

    DO_TOGGLE(GLFW_KEY_1, dirLightOn, key1_pressed)
        DO_TOGGLE(GLFW_KEY_2, pointLightOn, key2_pressed)
        DO_TOGGLE(GLFW_KEY_3, spotLightOn, key3_pressed)
        DO_TOGGLE(GLFW_KEY_5, ambientOn, key5_pressed)
        DO_TOGGLE(GLFW_KEY_6, diffuseOn, key6_pressed)
        DO_TOGGLE(GLFW_KEY_7, specularOn, key7_pressed)
        DO_TOGGLE(GLFW_KEY_L, masterLightOn, keyL_pressed)
        DO_TOGGLE(GLFW_KEY_M, mainLightOn, keyM_pressed)
        DO_TOGGLE(GLFW_KEY_G, fanOn, keyG_pressed)
        DO_TOGGLE(GLFW_KEY_V, singleViewport, keyV_pressed)
        DO_TOGGLE(GLFW_KEY_T, useTextureColorOnly, keyT_pressed)
        DO_TOGGLE(GLFW_KEY_P, usePhong, keyP_pressed)

        // We can leave 8 and 9 as backups if they were already there
        if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS) {
            if (!key8_pressed) { usePhong = true; key8_pressed = true; }
        }
        else { key8_pressed = false; }

    if (glfwGetKey(window, GLFW_KEY_9) == GLFW_PRESS) {
        if (!key9_pressed) { usePhong = false; key9_pressed = true; }
    }
    else { key9_pressed = false; }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // The screen is resized but viewports are calculated dynamically each frame
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    mainCamera.ProcessMouseMovement(xoffset, yoffset);
}

unsigned int loadTexture(char const* path, unsigned char r, unsigned char g, unsigned char b)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)      format = GL_RED;
        else if (nrComponents == 3) format = GL_RGB;
        else if (nrComponents == 4) format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);

        // Solid color fallback texture to distinguish materials
        unsigned char fallback[] = { r, g, b, 255 };
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, fallback);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }

    return textureID;
}
