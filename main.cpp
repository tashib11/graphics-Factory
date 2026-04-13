


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
const unsigned int SCR_WIDTH = 1900;
const unsigned int SCR_HEIGHT =950;

// cameras for 4 viewports
// Start outside the building, centred on the door gap (X=0),
// 120 units in front of the front wall (Z=100), at eye height Y=10.
// Yaw=-90 points in the -Z direction (straight at the door).
// Pitch=-5 gives a slight downward tilt so the entrance is well-framed.
Camera mainCamera(glm::vec3(0.0f, 10.0f, 220.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, -5.0f);
Camera birdEyeCamera(glm::vec3(0.0f, 20.0f, 0.1f));
Camera followCamera(glm::vec3(0.0f, 5.0f, 15.0f));
Camera frontCamera(glm::vec3(0.0f, 2.0f, 10.0f));

float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// Conveyor belt box speed (units/sec) - controllable via + / - keys
float conveyorSpeed = 3.0f;
float conveyorTexScroll = 0.0f;

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
bool p_override_light = false;
bool fanOn = false;
bool singleViewport = false;
bool useTextureColorOnly = false;
bool textureOn = true;
bool keyR_pressed = false;

// Debounce state
bool key1_pressed = false, key2_pressed = false, key3_pressed = false;
bool key5_pressed = false, key6_pressed = false, key7_pressed = false;
bool keyL_pressed = false, keyM_pressed = false, keyG_pressed = false;
bool key8_pressed = false, key9_pressed = false, keyV_pressed = false;
bool keyT_pressed = false;
bool keyP_pressed = false;
bool keyLeft_pressed = false;
bool keyRight_pressed = false;

// Animations
float fanAngle = 0.0f;
float robotBaseAngle = 0.0f;
float robotElbowAngle = 0.0f;
float bindRotAngle = 0.0f;
float armReachAngle = -30.0f;
float gripperSpread = 0.3f; // gripper finger spread (0.1=closed, 0.7=open)

float doorYOffset = 0.0f;
bool doorOpen = false;
bool keyO_pressed = false;

enum SegmentType { STRAIGHT, CURVE };
struct PathSegment {
    SegmentType type;
    float length;
    glm::vec3 start;
    glm::vec3 end;
    glm::vec3 center;
    float radius;
    float startAngle; // in radians
    float sweepAngle; // in radians
};
std::vector<PathSegment> globalPaths[5];

float getPathTotalLength(int pathIndex) {
    if (pathIndex < 0 || pathIndex >= 5) return 0;
    float len = 0;
    for(const auto& seg : globalPaths[pathIndex]) len += seg.length;
    return len;
}

void getPathPositionAndAngle(int pathIndex, float dist, glm::vec3& outPos, float& outAngle) {
    if (pathIndex < 0 || pathIndex >= 5 || globalPaths[pathIndex].empty()) return;
    float remaining = dist;
    for (const auto& seg : globalPaths[pathIndex]) {
        if (remaining <= seg.length) {
            float t = remaining / seg.length;
            if (seg.type == STRAIGHT) {
                outPos = glm::mix(seg.start, seg.end, t);
                glm::vec3 dir = glm::normalize(seg.end - seg.start);
                outAngle = atan2(dir.x, dir.z);
            } else {
                float currentAngle = seg.startAngle + t * seg.sweepAngle;
                outPos = seg.center + glm::vec3(cos(currentAngle)*seg.radius, seg.start.y - seg.center.y, sin(currentAngle)*seg.radius);
                // Angle of curve tangent
                float tangentAngle = currentAngle + (seg.sweepAngle > 0 ? glm::pi<float>()/2.0f : -glm::pi<float>()/2.0f);
                glm::vec3 dir(cos(tangentAngle), 0, sin(tangentAngle));
                outAngle = atan2(dir.x, dir.z);
            }
            return;
        }
        remaining -= seg.length;
    }
    // If beyond, cap to end
    const auto& seg = globalPaths[pathIndex].back();
    outPos = seg.end;
    glm::vec3 dir = (seg.type == STRAIGHT) ? glm::normalize(seg.end - seg.start) : glm::vec3(cos(seg.startAngle + seg.sweepAngle + (seg.sweepAngle > 0 ? glm::pi<float>()/2.0f : -glm::pi<float>()/2.0f)), 0, sin(seg.startAngle + seg.sweepAngle + (seg.sweepAngle > 0 ? glm::pi<float>()/2.0f : -glm::pi<float>()/2.0f)));
    outAngle = atan2(dir.x, dir.z);
}

enum BoxStage { RAW = 0, PAINTED = 1, BOUND = 2 };
enum BoxState { ON_BELT, WAITING_FOR_PICKUP, BEING_PICKED, ON_SHELF };

struct GridBox {
    float distance;
    BoxStage stage;
    BoxState state = ON_BELT;
    glm::vec3 worldPos = glm::vec3(0.0f);
    int shelfSide = 0; // 0=left(source), 1=right(dest)
    int shelfTower = 0;
    int shelfTier = 0;
    int shelfSlot = 0;
    float pickTimer = 0.0f;
    int pathIndex = 0; // Track which of the 5 belts this box is on
};
std::vector<GridBox> gridBoxes;

// Shelf tracking: [Side: 0=Left, 1=Right][Tower][Tier][Slot]
const int SHELF_TOWERS = 10;
const int SHELF_TIERS = 5;
const int SHELF_SLOTS = 9;
bool shelfOccupied[2][SHELF_TOWERS][SHELF_TIERS][SHELF_SLOTS]; 
glm::vec3 shelfSlotPos[2][SHELF_TOWERS][SHELF_TIERS][SHELF_SLOTS];


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
ShelfArm shelfArms[20];

struct Particle {
    glm::vec3 position;
    glm::vec3 velocity;
    float life; 
    float initialLife;
};
std::vector<Particle> sparkParticles;

void spawnSparks(glm::vec3 pos, int count) {
    for (int i = 0; i < count; i++) {
        Particle p;
        p.position = pos;
        float rx = ((rand() % 100) / 50.0f) - 1.0f; // -1 to 1
        float ry = ((rand() % 100) / 100.0f) + 0.5f; // 0.5 to 1.5
        float rz = ((rand() % 100) / 50.0f) - 1.0f; // -1 to 1
        p.velocity = glm::vec3(rx, ry, rz) * 4.0f; 
        p.life = 0.5f + (rand() % 100) / 200.0f;
        p.initialLife = p.life;
        sparkParticles.push_back(p);
    }
}

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

    // --- Small pendant lights in a grid across the full ceiling ---
    for (float x = -80.0f; x <= 80.0f; x += 20.0f) {
        for (float z = -80.0f; z <= 80.0f; z += 20.0f) {
            // Ceiling mount plate
            glBindTexture(GL_TEXTURE_2D, lightTex);
            glm::mat4 plate = glm::translate(glm::mat4(1.0f), glm::vec3(x, 29.85f, z));
            plate = glm::scale(plate, glm::vec3(0.6f, 0.3f, 0.6f));
            shader.setMat4("model", plate);
            glDrawArrays(GL_TRIANGLES, 0, 36);

            // Drop cord (thin vertical rod)
            glBindTexture(GL_TEXTURE_2D, lightTex);
            glm::mat4 cord = glm::translate(glm::mat4(1.0f), glm::vec3(x, 28.5f, z));
            cord = glm::scale(cord, glm::vec3(0.07f, 2.5f, 0.07f));
            shader.setMat4("model", cord);
            glDrawArrays(GL_TRIANGLES, 0, 36);

            // Lamp housing (rectangular shade body, wider bottom)
            glBindTexture(GL_TEXTURE_2D, lightTex);
            glm::mat4 shade = glm::translate(glm::mat4(1.0f), glm::vec3(x, 27.1f, z));
            shade = glm::scale(shade, glm::vec3(1.6f, 0.5f, 1.6f));
            shader.setMat4("model", shade);
            glDrawArrays(GL_TRIANGLES, 0, 36);

            // Glowing bottom panel (emissive yellow)
            glBindTexture(GL_TEXTURE_2D, yellowTex);
            glm::mat4 glow = glm::translate(glm::mat4(1.0f), glm::vec3(x, 26.84f, z));
            glow = glm::scale(glow, glm::vec3(1.5f, 0.08f, 1.5f));
            shader.setMat4("model", glow);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // --- 4 Large industrial pendant lights at point-light positions ---
    float bigX[] = { -60.0f,  60.0f, -60.0f, 60.0f };
    float bigZ[] = {  60.0f,  60.0f, -60.0f, -60.0f };
    for (int i = 0; i < 4; i++) {
        float px = bigX[i], pz = bigZ[i];

        // Ceiling mount
        glBindTexture(GL_TEXTURE_2D, lightTex);
        glm::mat4 mount = glm::translate(glm::mat4(1.0f), glm::vec3(px, 29.8f, pz));
        mount = glm::scale(mount, glm::vec3(1.2f, 0.4f, 1.2f));
        shader.setMat4("model", mount);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Thick cord/chain
        glm::mat4 cord = glm::translate(glm::mat4(1.0f), glm::vec3(px, 27.5f, pz));
        cord = glm::scale(cord, glm::vec3(0.12f, 4.5f, 0.12f));
        shader.setMat4("model", cord);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Wide conical housing (top narrower, bottom wider — approximate with two boxes)
        glm::mat4 hTop = glm::translate(glm::mat4(1.0f), glm::vec3(px, 25.55f, pz));
        hTop = glm::scale(hTop, glm::vec3(1.8f, 0.6f, 1.8f));
        shader.setMat4("model", hTop);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        glm::mat4 hBot = glm::translate(glm::mat4(1.0f), glm::vec3(px, 25.0f, pz));
        hBot = glm::scale(hBot, glm::vec3(2.6f, 0.4f, 2.6f));
        shader.setMat4("model", hBot);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Large bright emissive bottom disc
        glBindTexture(GL_TEXTURE_2D, yellowTex);
        glm::mat4 glow = glm::translate(glm::mat4(1.0f), glm::vec3(px, 24.78f, pz));
        glow = glm::scale(glow, glm::vec3(2.5f, 0.1f, 2.5f));
        shader.setMat4("model", glow);
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


glm::vec3 evaluateBSpline(float t, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) {
    float it = 1.0f - t;
    float t2 = t * t;
    float t3 = t2 * t;
    
    float b0 = (it * it * it) / 6.0f;
    float b1 = (3.0f * t3 - 6.0f * t2 + 4.0f) / 6.0f;
    float b2 = (-3.0f * t3 + 3.0f * t2 + 3.0f * t + 1.0f) / 6.0f;
    float b3 = t3 / 6.0f;
    
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

glm::vec3 evaluateBSplineDerivative(float t, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) {
    float t2 = t * t;
    
    float d0 = -0.5f * (1.0f - t) * (1.0f - t);
    float d1 = 1.5f * t2 - 2.0f * t;
    float d2 = -1.5f * t2 + t + 0.5f;
    float d3 = 0.5f * t2;
    
    return p0 * d0 + p1 * d1 + p2 * d2 + p3 * d3;
}

// ─── Helicoid Slide ─────────────────────────────────────────────────────────
unsigned int helicoidVAO = 0;
int helicoidVertexCount = 0;
unsigned int watchTexture = 0;
unsigned int fracTexture = 0;

void drawFractalPillar(Shader& shader, glm::mat4 baseModel, int depth, float length, float thickness, float angleDeg, float phaseAngle = 0.0f) {
    if (depth <= 0) return;

    // Draw the trunk for this iteration
    glm::mat4 segModel = glm::translate(baseModel, glm::vec3(0.0f, length * 0.5f, 0.0f));
    segModel = glm::scale(segModel, glm::vec3(thickness, length, thickness));
    shader.setMat4("model", segModel);
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Tip of this trunk
    glm::mat4 tipModel = glm::translate(baseModel, glm::vec3(0.0f, length, 0.0f));

    // For the next branches, shrink thickness
    float newThickness = thickness * 0.7f;
    
    // Left branch
    glm::mat4 leftM = glm::rotate(tipModel, glm::radians(angleDeg), glm::vec3(cos(phaseAngle), 0.0f, sin(phaseAngle)));
    float leftLen = length * 0.65f;
    // Mathematically constrain exactly 2 splits to reach the 30.0 ceiling perfectly
    if (depth == 3) leftLen = 5.5f; // Calibrate middle branch length visually
    if (depth == 2 && leftM[1][1] > 0.01f) leftLen = (30.0f - leftM[3][1]) / leftM[1][1]; // Snap final branch precisely to Y=30.0
    
    drawFractalPillar(shader, leftM, depth - 1, leftLen, newThickness, angleDeg * 0.95f, phaseAngle + 1.57f);
    
    // Right branch
    glm::mat4 rightM = glm::rotate(tipModel, glm::radians(-angleDeg), glm::vec3(cos(phaseAngle), 0.0f, sin(phaseAngle)));
    float rightLen = length * 0.65f;
    if (depth == 3) rightLen = 5.5f;
    if (depth == 2 && rightM[1][1] > 0.01f) rightLen = (30.0f - rightM[3][1]) / rightM[1][1];
    
    drawFractalPillar(shader, rightM, depth - 1, rightLen, newThickness, angleDeg * 0.95f, phaseAngle + 1.57f);
}

void buildHelicoidSlide() {
    // Helicoid parametric surface:
    //   P(u,v) = ( u*cos(v),  c*v,  u*sin(v) )
    // where u in [R_inner, R_outer], v in [0, totalAngle]
    // The model-matrix will translate this to world pos (70, 0, 70).

    const float R_inner    = 0.8f;   // tight inner radius — hugs the central pillar
    const float R_outer    = 7.8f;   // wider walkable surface
    const float H_total    = 21.0f;  // height (Y=0 floor -> Y=21 top) for taller walkable slope
    const float numTurns   = 4.0f;   // FEWER turns = significantly larger pitch for pedestrian clearance
    const float totalAngle = numTurns * 2.0f * 3.14159265f;
    const float c          = H_total / totalAngle; // pitch constant

    const int uSeg = 24;  // radial subdivisions
    const int vSeg = 120; // angular subdivisions — smooth continuous spiral

    std::vector<float> verts;
    verts.reserve(uSeg * vSeg * 6 * 8 * 2); // 2 sides for solid look

    // Analytic normal of the helicoid:
    //   dP/du = (  cos(v),     0,   sin(v) )
    //   dP/dv = ( -u*sin(v),   c,   u*cos(v) )
    //   N = dP/du x dP/dv = ( -c*sin(v), -u,  c*cos(v) )  — then normalize
    auto hPoint = [&](float u, float v) -> glm::vec3 {
        return glm::vec3(u * cosf(v), c * v, u * sinf(v));
    };
    auto hNormal = [&](float u, float v) -> glm::vec3 {
        glm::vec3 n(-c * sinf(v), -u, c * cosf(v));
        float len = glm::length(n);
        return (len > 1e-6f) ? n / len : glm::vec3(0,1,0);
    };

    auto push = [&](float u, float v, bool flip) {
        glm::vec3 p = hPoint(u, v);
        glm::vec3 n = hNormal(u, v);
        if (flip) n = -n;
        float tu = (u - R_inner) / (R_outer - R_inner);
        float tv = v / totalAngle;
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        verts.push_back(n.x); verts.push_back(n.y); verts.push_back(n.z);
        verts.push_back(tu);  verts.push_back(tv);
    };

    for (int vi = 0; vi < vSeg; vi++) {
        float v0 = totalAngle * (float)vi       / vSeg;
        float v1 = totalAngle * (float)(vi + 1) / vSeg;
        for (int ui = 0; ui < uSeg; ui++) {
            float u0 = R_inner + (R_outer - R_inner) * (float)ui       / uSeg;
            float u1 = R_inner + (R_outer - R_inner) * (float)(ui + 1) / uSeg;

            // Top face (normal pointing up/outward)
            push(u0, v0, false); push(u1, v0, false); push(u0, v1, false);
            push(u1, v0, false); push(u1, v1, false); push(u0, v1, false);
            // Bottom face (flipped — so underside is also solid/shaded)
            push(u0, v0, true);  push(u0, v1, true);  push(u1, v0, true);
            push(u1, v0, true);  push(u0, v1, true);  push(u1, v1, true);
        }
    }

    glGenVertexArrays(1, &helicoidVAO);
    unsigned int vbo;
    glGenBuffers(1, &vbo);
    glBindVertexArray(helicoidVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    helicoidVertexCount = (int)(verts.size() / 8);
}

unsigned int ductVAO = 0;
int ductVertexCount = 0;
unsigned int ductTexture = 0;
unsigned int streetTexture = 0;
unsigned int grassTexture = 0;
unsigned int skyTexture = 0;
unsigned int barrelTexture = 0;
unsigned int barrelVAO = 0;
int barrelVertexCount = 0;

void buildDuctworkSystem() {
    std::vector<float> ductVerts;
    float tubeRadius = 0.8f; // Thickened so components remain intimately combined/mixed
    int radialSegments = 16;
    int segmentsPerCurve = 30;

    for (int b = 0; b < 5; b++) {
        float bz = -40.0f + b * 20.0f;
        float R = 40.0f;
        float beltY = (b % 2 == 0) ? 0.8f : 4.5f;

        // The Paint Chambers are exactly halfway through the curve.
        // Even belts curve towards +Z (sweep = -PI), Mid = bz + R
        // Odd belts curve towards -Z (sweep = PI), Mid = bz - R
        float chamberZ = (b % 2 == 0) ? bz + R : bz - R; 
        
        // Maintain consistent distinct spacing to prevent bundled connections
        float pathZ = chamberZ;
        
        std::vector<glm::vec3> cp;
        // Extend sequence at ends via duplicates for B-spline completeness
        // Start exactly centered on the Paint Chamber. Tunnel top is at beltY + 1.7f.
        glm::vec3 startP(0.0f, beltY + 1.0f, pathZ);
        cp.push_back(startP); 
        cp.push_back(startP); 
        cp.push_back(startP); 
        
        // Ascend vertically out of the chamber
        cp.push_back(glm::vec3(0.0f, 16.0f, pathZ));
        cp.push_back(glm::vec3(0.0f, 25.0f, pathZ));
        
        // Bend smoothly towards the back wall without merging
        cp.push_back(glm::vec3(0.0f, 28.0f, pathZ));
        cp.push_back(glm::vec3(-20.0f, 28.0f, pathZ)); 
        cp.push_back(glm::vec3(-40.0f, 28.0f, pathZ)); 
        cp.push_back(glm::vec3(-70.0f, 28.0f, pathZ));
        
        glm::vec3 endP(-100.0f, 28.0f, pathZ);
        cp.push_back(endP);
        cp.push_back(endP);
        cp.push_back(endP);

        // Initialize our Parallel Transport Frame BEFORE the loop to maintain a continuous, twisting-free extrusion
        glm::vec3 currentUp(1.0f, 0.0f, 0.0f); 

        for (size_t i = 0; i < cp.size() - 3; i++) {
            const glm::vec3& p0 = cp[i];
            const glm::vec3& p1 = cp[i+1];
            const glm::vec3& p2 = cp[i+2];
            const glm::vec3& p3 = cp[i+3];

            for (int s = 0; s < segmentsPerCurve; s++) {
                float t1 = (float)s / segmentsPerCurve;
                float t2 = (float)(s + 1) / segmentsPerCurve;

                glm::vec3 c1 = evaluateBSpline(t1, p0, p1, p2, p3);
                glm::vec3 c2 = evaluateBSpline(t2, p0, p1, p2, p3);

                glm::vec3 d1 = evaluateBSplineDerivative(t1, p0, p1, p2, p3);
                glm::vec3 d2 = evaluateBSplineDerivative(t2, p0, p1, p2, p3);

                // FIX FOR NaN "GAPS/HOLES": Check length BEFORE normalizing!
                // Endpoints have duplicated CPs causing 0-length derivatives
                if (glm::length(d1) < 0.0001f) d1 = (p0.y < 20.0f) ? glm::vec3(0, 1, 0) : glm::vec3(-1, 0, 0);
                if (glm::length(d2) < 0.0001f) d2 = (p3.y < 20.0f) ? glm::vec3(0, 1, 0) : glm::vec3(-1, 0, 0);

                glm::vec3 tan1 = glm::normalize(d1);
                glm::vec3 tan2 = glm::normalize(d2);

                glm::vec3 right1 = glm::cross(tan1, currentUp);
                if (glm::length(right1) < 0.001f) {
                    currentUp = glm::vec3(0.0f, 0.0f, 1.0f);
                    right1 = glm::cross(tan1, currentUp);
                    if (glm::length(right1) < 0.001f) right1 = glm::cross(tan1, glm::vec3(0.0f, 1.0f, 0.0f));
                }
                right1 = glm::normalize(right1);
                glm::vec3 up1 = glm::normalize(glm::cross(right1, tan1));
                
                // Propagate frame to evaluate the second ring properly
                currentUp = up1;

                glm::vec3 right2 = glm::cross(tan2, currentUp);
                if (glm::length(right2) < 0.001f) {
                    right2 = glm::cross(tan2, glm::vec3(0.0f, 0.0f, 1.0f));
                    if (glm::length(right2) < 0.001f) right2 = glm::cross(tan2, glm::vec3(0.0f, 1.0f, 0.0f));
                }
                right2 = glm::normalize(right2);
                glm::vec3 up2 = glm::normalize(glm::cross(right2, tan2));
                
                // CRUCIAL BUG FIX: Propagate the frame UP vector for the next sub-segment (s+1) 
                // so the quad rings perfectly snap together without tearing gaps between iterations!
                currentUp = up2;

                for (int r = 0; r < radialSegments; r++) {
                    float a1 = 2.0f * 3.14159f * (float)r / radialSegments;
                    float a2 = 2.0f * 3.14159f * (float)(r+1) / radialSegments;

                    float cos1 = cos(a1), sin1 = sin(a1);
                    float cos2 = cos(a2), sin2 = sin(a2);

                    glm::vec3 n11 = cos1 * right1 + sin1 * up1;
                    glm::vec3 n12 = cos2 * right1 + sin2 * up1;
                    glm::vec3 n21 = cos1 * right2 + sin1 * up2;
                    glm::vec3 n22 = cos2 * right2 + sin2 * up2;

                    glm::vec3 p11 = c1 + tubeRadius * n11;
                    glm::vec3 p12 = c1 + tubeRadius * n12;
                    glm::vec3 p21 = c2 + tubeRadius * n21;
                    glm::vec3 p22 = c2 + tubeRadius * n22;

                    // UV mapping
                    float u1 = (float)r / radialSegments * 4.0f;
                    float u2 = (float)(r+1) / radialSegments * 4.0f;
                    float v1 = (float)(i * segmentsPerCurve + s) / 5.0f;
                    float v2 = (float)(i * segmentsPerCurve + s + 1) / 5.0f;

                    auto pushV = [&](glm::vec3 p, glm::vec3 n, float u, float v) {
                        ductVerts.push_back(p.x); ductVerts.push_back(p.y); ductVerts.push_back(p.z);
                        ductVerts.push_back(n.x); ductVerts.push_back(n.y); ductVerts.push_back(n.z);
                        ductVerts.push_back(u); ductVerts.push_back(v);
                    };

                    pushV(p11, n11, u1, v1);
                    pushV(p12, n12, u2, v1);
                    pushV(p21, n21, u1, v2);

                    pushV(p12, n12, u2, v1);
                    pushV(p22, n22, u2, v2);
                    pushV(p21, n21, u1, v2);
                }
            }
        }
    }

    glGenVertexArrays(1, &ductVAO);
    unsigned int vbo;
    glGenBuffers(1, &vbo);
    glBindVertexArray(ductVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, ductVerts.size() * sizeof(float), &ductVerts[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    ductVertexCount = ductVerts.size() / 8;
}

std::vector<glm::mat4> catwalkStanchions;
unsigned int cwFloorVAO = 0;
int cwFloorVertexCount = 0;
unsigned int cwRailVAO = 0;
int cwRailVertexCount = 0;

void buildCatwalkSystem() {
    std::vector<glm::vec3> cp = {
        glm::vec3(-100.0f, 16.5f, -15.0f), 
        glm::vec3(-70.0f, 16.5f, -15.0f),
        glm::vec3(  10.0f, 16.5f, -15.0f),
        glm::vec3( 45.0f, 16.5f, -15.0f),
        glm::vec3( 65.0f, 16.5f, -15.0f), 
        glm::vec3( 90.0f, 16.5f,  20.0f), 
        glm::vec3( 65.0f, 16.5f,  55.0f), 
        glm::vec3( 40.0f, 16.5f,  55.0f), 
        glm::vec3( 10.0f, 11.5f,  55.0f), 
        glm::vec3(-30.0f,  6.5f,  55.0f),
        glm::vec3(-70.0f,  0.5f,  55.0f), 
        glm::vec3(-100.0f, 0.5f,  55.0f)
    };

    std::vector<float> floorVerts;
    std::vector<float> railVerts;

    float catwalkWidth = 2.4f;
    float railHeight = 1.6f;
    float tubeRadius = 0.15f;
    int segmentsPerCurve = 25;
    int radialSegments = 12;
    catwalkStanchions.clear();

    for (size_t i = 0; i < cp.size() - 3; i++) {
        const glm::vec3& p0 = cp[i];
        const glm::vec3& p1 = cp[i+1];
        const glm::vec3& p2 = cp[i+2];
        const glm::vec3& p3 = cp[i+3];

        for (int s = 0; s < segmentsPerCurve; s++) {
            float t1 = (float)s / segmentsPerCurve;
            float t2 = (float)(s + 1) / segmentsPerCurve;

            glm::vec3 c1 = evaluateBSpline(t1, p0, p1, p2, p3);
            glm::vec3 c2 = evaluateBSpline(t2, p0, p1, p2, p3);

            glm::vec3 tan1 = glm::normalize(evaluateBSplineDerivative(t1, p0, p1, p2, p3));
            glm::vec3 tan2 = glm::normalize(evaluateBSplineDerivative(t2, p0, p1, p2, p3));

            glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
            glm::vec3 right1 = glm::normalize(glm::cross(tan1, worldUp));
            glm::vec3 right2 = glm::normalize(glm::cross(tan2, worldUp));

            glm::vec3 up1 = glm::normalize(glm::cross(right1, tan1));
            glm::vec3 up2 = glm::normalize(glm::cross(right2, tan2));

            glm::vec3 flL1 = c1 - right1 * catwalkWidth;
            glm::vec3 flR1 = c1 + right1 * catwalkWidth;
            glm::vec3 flL2 = c2 - right2 * catwalkWidth;
            glm::vec3 flR2 = c2 + right2 * catwalkWidth;

            float v_coord1 = (float)(i * segmentsPerCurve + s) / 2.0f;
            float v_coord2 = (float)(i * segmentsPerCurve + s + 1) / 2.0f;

            floorVerts.push_back(flL1.x); floorVerts.push_back(flL1.y); floorVerts.push_back(flL1.z);
            floorVerts.push_back(up1.x); floorVerts.push_back(up1.y); floorVerts.push_back(up1.z);
            floorVerts.push_back(0.0f); floorVerts.push_back(v_coord1);
            floorVerts.push_back(flR1.x); floorVerts.push_back(flR1.y); floorVerts.push_back(flR1.z);
            floorVerts.push_back(up1.x); floorVerts.push_back(up1.y); floorVerts.push_back(up1.z);
            floorVerts.push_back(1.0f); floorVerts.push_back(v_coord1);
            floorVerts.push_back(flL2.x); floorVerts.push_back(flL2.y); floorVerts.push_back(flL2.z);
            floorVerts.push_back(up2.x); floorVerts.push_back(up2.y); floorVerts.push_back(up2.z);
            floorVerts.push_back(0.0f); floorVerts.push_back(v_coord2);

            floorVerts.push_back(flR1.x); floorVerts.push_back(flR1.y); floorVerts.push_back(flR1.z);
            floorVerts.push_back(up1.x); floorVerts.push_back(up1.y); floorVerts.push_back(up1.z);
            floorVerts.push_back(1.0f); floorVerts.push_back(v_coord1);
            floorVerts.push_back(flR2.x); floorVerts.push_back(flR2.y); floorVerts.push_back(flR2.z);
            floorVerts.push_back(up2.x); floorVerts.push_back(up2.y); floorVerts.push_back(up2.z);
            floorVerts.push_back(1.0f); floorVerts.push_back(v_coord2);
            floorVerts.push_back(flL2.x); floorVerts.push_back(flL2.y); floorVerts.push_back(flL2.z);
            floorVerts.push_back(up2.x); floorVerts.push_back(up2.y); floorVerts.push_back(up2.z);
            floorVerts.push_back(0.0f); floorVerts.push_back(v_coord2);

            glm::vec3 rBaseL1 = flL1 + up1 * railHeight;
            glm::vec3 rBaseL2 = flL2 + up2 * railHeight;
            glm::vec3 rBaseR1 = flR1 + up1 * railHeight;
            glm::vec3 rBaseR2 = flR2 + up2 * railHeight;

            auto pushVert = [&](std::vector<float>& arr, glm::vec3 p, glm::vec3 n, float u, float v) {
                arr.push_back(p.x); arr.push_back(p.y); arr.push_back(p.z);
                arr.push_back(n.x); arr.push_back(n.y); arr.push_back(n.z);
                arr.push_back(u); arr.push_back(v);
            };

            for (int r = 0; r <= radialSegments; r++) {
                float angle = 2.0f * 3.14159f * (float)r / radialSegments;
                float cosA = cos(angle); float sinA = sin(angle);
                float nextAngle = 2.0f * 3.14159f * (float)(r+1) / radialSegments;
                float ncosA = cos(nextAngle); float nsinA = sin(nextAngle);

                float u1 = (float)r / radialSegments;
                float u2 = (float)(r+1) / radialSegments;

                // LEFT RAIL
                glm::vec3 pTL = rBaseL1 + tubeRadius * (cosA * right1 + sinA * up1);
                glm::vec3 nTL = glm::normalize(pTL - rBaseL1);
                glm::vec3 pBL = rBaseL2 + tubeRadius * (cosA * right2 + sinA * up2);
                glm::vec3 nBL = glm::normalize(pBL - rBaseL2);
                glm::vec3 pTR = rBaseL1 + tubeRadius * (ncosA * right1 + nsinA * up1);
                glm::vec3 nTR = glm::normalize(pTR - rBaseL1);
                glm::vec3 pBR = rBaseL2 + tubeRadius * (ncosA * right2 + nsinA * up2);
                glm::vec3 nBR = glm::normalize(pBR - rBaseL2);

                pushVert(railVerts, pTL, nTL, u1, v_coord1); pushVert(railVerts, pTR, nTR, u2, v_coord1); pushVert(railVerts, pBL, nBL, u1, v_coord2);
                pushVert(railVerts, pTR, nTR, u2, v_coord1); pushVert(railVerts, pBR, nBR, u2, v_coord2); pushVert(railVerts, pBL, nBL, u1, v_coord2);

                // RIGHT RAIL
                glm::vec3 rpTL = rBaseR1 + tubeRadius * (cosA * right1 + sinA * up1);
                glm::vec3 rnTL = glm::normalize(rpTL - rBaseR1);
                glm::vec3 rpBL = rBaseR2 + tubeRadius * (cosA * right2 + sinA * up2);
                glm::vec3 rnBL = glm::normalize(rpBL - rBaseR2);
                glm::vec3 rpTR = rBaseR1 + tubeRadius * (ncosA * right1 + nsinA * up1);
                glm::vec3 rnTR = glm::normalize(rpTR - rBaseR1);
                glm::vec3 rpBR = rBaseR2 + tubeRadius * (ncosA * right2 + nsinA * up2);
                glm::vec3 rnBR = glm::normalize(rpBR - rBaseR2);

                pushVert(railVerts, rpTL, rnTL, u1, v_coord1); pushVert(railVerts, rpTR, rnTR, u2, v_coord1); pushVert(railVerts, rpBL, rnBL, u1, v_coord2);
                pushVert(railVerts, rpTR, rnTR, u2, v_coord1); pushVert(railVerts, rpBR, rnBR, u2, v_coord2); pushVert(railVerts, rpBL, rnBL, u1, v_coord2);
            }

            if (s % 6 == 0) {
                float mLen = railHeight;
                for (int side = 0; side < 2; side++) {
                    glm::vec3 basePt = (side == 0) ? flL1 : flR1;
                    glm::vec3 midPt = basePt + up1 * (railHeight * 0.5f);
                    glm::mat4 sMat = glm::translate(glm::mat4(1.0f), midPt);
                    glm::vec3 yAxis(0, 1, 0);
                    if (glm::length(glm::cross(yAxis, up1)) > 0.001f) {
                        float angle = acos(glm::dot(yAxis, up1));
                        glm::vec3 axis = glm::normalize(glm::cross(yAxis, up1));
                        sMat = glm::rotate(sMat, angle, axis);
                    }
                    sMat = glm::scale(sMat, glm::vec3(0.08f, mLen, 0.08f));
                    catwalkStanchions.push_back(sMat);
                }
            }
        }
    }

    cwFloorVertexCount = floorVerts.size() / 8;
    unsigned int vboF;
    glGenVertexArrays(1, &cwFloorVAO);
    glGenBuffers(1, &vboF);
    glBindVertexArray(cwFloorVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vboF);
    glBufferData(GL_ARRAY_BUFFER, floorVerts.size() * sizeof(float), floorVerts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    
    cwRailVertexCount = railVerts.size() / 8;
    unsigned int vboR;
    glGenVertexArrays(1, &cwRailVAO);
    glGenBuffers(1, &vboR);
    glBindVertexArray(cwRailVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vboR);
    glBufferData(GL_ARRAY_BUFFER, railVerts.size() * sizeof(float), railVerts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
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

unsigned int cylinderVAO = 0;
int cylinderVertexCount = 0;

unsigned int fanBladeVAO = 0;
int fanBladeVertexCount = 0;
unsigned int fanTexture = 0;
float exhaustFanSpeed = 10.0f;
float exhaustFanAngle = 0.0f;

void buildFanBlade() {
    std::vector<float> verts;
    int uSegments = 20;
    int vSegments = 20;
    
    // Industrial exhaust blades are long, swept, and have deep root chords.
    float R_root = 0.5f;
    float R_tip = 3.9f; // Fits cleanly inside our massive R=4 wall hole 
    float C_root = 1.8f;
    float C_tip = 0.8f;
    float A_root = 60.0f * (3.14159f / 180.0f);
    float A_tip = 25.0f * (3.14159f / 180.0f);
    float Sweep = -1.0f; 
    float MaxCamber = 0.4f;

    auto getPoint = [&](float t_u, float t_v) -> glm::vec3 {
        float u = t_u - 0.5f; 
        float radius = R_root + t_v * (R_tip - R_root);
        float chord = C_root + t_v * (C_tip - C_root);
        float angle = A_root + t_v * (A_tip - A_root);
        float sweep = Sweep * t_v * t_v; 
        
        float camber = MaxCamber * (1.0f - 4.0f * u * u) * (1.0f - t_v * 0.5f); 
        
        float z = u * chord * cos(angle) + sweep;
        float x = u * chord * sin(angle) + camber;
        return glm::vec3(x, radius, z);
    };

    auto getNormal = [&](float t_u, float t_v) -> glm::vec3 {
        float eps = 0.01f;
        glm::vec3 pu1 = getPoint(glm::clamp(t_u + eps, 0.0f, 1.0f), t_v);
        glm::vec3 pu0 = getPoint(glm::clamp(t_u - eps, 0.0f, 1.0f), t_v);
        glm::vec3 pv1 = getPoint(t_u, glm::clamp(t_v + eps, 0.0f, 1.0f));
        glm::vec3 pv0 = getPoint(t_u, glm::clamp(t_v - eps, 0.0f, 1.0f));
        glm::vec3 du = glm::normalize(pu1 - pu0);
        glm::vec3 dv = glm::normalize(pv1 - pv0);
        return glm::normalize(glm::cross(du, dv)); 
    };

    auto pushV = [&](float tu, float tv) {
        glm::vec3 p = getPoint(tu, tv);
        glm::vec3 n = getNormal(tu, tv);
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        verts.push_back(n.x); verts.push_back(n.y); verts.push_back(n.z);
        verts.push_back(tu); verts.push_back(tv);
    };
    auto pushV_B = [&](float tu, float tv) {
        glm::vec3 p = getPoint(tu, tv);
        glm::vec3 n = -getNormal(tu, tv);
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        verts.push_back(n.x); verts.push_back(n.y); verts.push_back(n.z);
        verts.push_back(tu); verts.push_back(tv);
    };

    for (int v = 0; v < vSegments; v++) {
        for (int u = 0; u < uSegments; u++) {
            float u1 = (float)u / uSegments;
            float u2 = (float)(u + 1) / uSegments;
            float v1 = (float)v / vSegments;
            float v2 = (float)(v + 1) / vSegments;

            pushV(u1, v1);
            pushV(u2, v1);
            pushV(u1, v2);

            pushV(u2, v1);
            pushV(u2, v2);
            pushV(u1, v2);
            
            // Flipped backfaces for solid two-sided culling capabilities
            pushV_B(u1, v1);
            pushV_B(u1, v2);
            pushV_B(u2, v1);
            
            pushV_B(u2, v1);
            pushV_B(u1, v2);
            pushV_B(u2, v2);
        }
    }

    glGenVertexArrays(1, &fanBladeVAO);
    unsigned int vbo;
    glGenBuffers(1, &vbo);
    glBindVertexArray(fanBladeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), &verts[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6*sizeof(float)));
    glEnableVertexAttribArray(2);
    
    fanBladeVertexCount = verts.size() / 8;
}
void buildCylinder() {
    std::vector<float> verts;
    int segments = 36;
    float radius = 0.5f;
    float length = 1.0f;
    
    // Side surface
    for (int i = 0; i <= segments; i++) {
        float theta = 2.0f * glm::pi<float>() * (float)i / (float)segments;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        float nx = cos(theta);
        float nz = sin(theta);
        float u = (float)i / segments;
        
        verts.push_back(x); verts.push_back(length / 2); verts.push_back(z);
        verts.push_back(nx); verts.push_back(0.0f); verts.push_back(nz);
        verts.push_back(u); verts.push_back(1.0f);
        
        verts.push_back(x); verts.push_back(-length / 2); verts.push_back(z);
        verts.push_back(nx); verts.push_back(0.0f); verts.push_back(nz);
        verts.push_back(u); verts.push_back(0.0f);
    }
    // Top and Bottom Caps could be added here, but for rollers/barrels sides are enough or we can add caps if needed.
    // For simplicity, we just use the side shell.
    
    cylinderVertexCount = (segments + 1) * 2;
    unsigned int VBO;
    glGenVertexArrays(1, &cylinderVAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(cylinderVAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
}

// ─── Proper Barrel Shape (bulging sides + closed caps + metal bands) ─────────
void buildBarrel() {
    std::vector<float> verts;
    const int   radialSegs = 32;
    const int   heightSegs = 24;
    const float PI   = 3.14159265f;
    const float r_top = 0.36f;   // radius at top & bottom rims
    const float r_mid = 0.50f;   // maximum radius at barrel equator

    // radius at normalised height t in [0,1]
    auto getR = [&](float t) { return r_top + (r_mid - r_top) * sinf(PI * t); };

    // push one vertex  pos | analytic-normal | uv
    auto push = [&](float r, float y, float cosA, float sinA,
                    float nR, float nY, float u, float v) {
        verts.push_back(r * cosA); verts.push_back(y);       verts.push_back(r * sinA);
        verts.push_back(nR * cosA); verts.push_back(nY); verts.push_back(nR * sinA);
        verts.push_back(u); verts.push_back(v);
    };

    // ── SIDE SURFACE ───────────────────────────────────────────────────────────
    for (int hi = 0; hi < heightSegs; hi++) {
        float t0 = (float)hi       / heightSegs;
        float t1 = (float)(hi + 1) / heightSegs;
        float y0 = -0.5f + t0,  y1 = -0.5f + t1;
        float r0 = getR(t0),    r1 = getR(t1);

        // Analytic outward normal from profile tangent (dR/dt, 1) → normal (1, -dR/dt) normalised
        float dR0 = (r_mid - r_top) * PI * cosf(PI * t0);
        float dR1 = (r_mid - r_top) * PI * cosf(PI * t1);
        float len0 = sqrtf(1.0f + dR0 * dR0), len1 = sqrtf(1.0f + dR1 * dR1);
        float nr0 = 1.0f / len0,  ny0 = -dR0 / len0;
        float nr1 = 1.0f / len1,  ny1 = -dR1 / len1;

        for (int ri = 0; ri < radialSegs; ri++) {
            float a0 = 2.0f * PI * (float)ri       / radialSegs;
            float a1 = 2.0f * PI * (float)(ri + 1) / radialSegs;
            float c0 = cosf(a0), s0 = sinf(a0);
            float c1 = cosf(a1), s1 = sinf(a1);
            float u0 = (float)ri / radialSegs, u1 = (float)(ri + 1) / radialSegs;

            push(r0, y0, c0, s0, nr0, ny0, u0, t0);
            push(r0, y0, c1, s1, nr0, ny0, u1, t0);
            push(r1, y1, c0, s0, nr1, ny1, u0, t1);

            push(r0, y0, c1, s1, nr0, ny0, u1, t0);
            push(r1, y1, c1, s1, nr1, ny1, u1, t1);
            push(r1, y1, c0, s0, nr1, ny1, u0, t1);
        }
    }

    // ── TOP CAP ────────────────────────────────────────────────────────────────
    {
        float y = 0.5f,  r = getR(1.0f);
        for (int ri = 0; ri < radialSegs; ri++) {
            float a0 = 2.0f * PI * (float)ri       / radialSegs;
            float a1 = 2.0f * PI * (float)(ri + 1) / radialSegs;
            // centre
            verts.push_back(0.0f); verts.push_back(y); verts.push_back(0.0f);
            verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f);
            verts.push_back(0.5f); verts.push_back(0.5f);
            // rim a0
            verts.push_back(r*cosf(a0)); verts.push_back(y); verts.push_back(r*sinf(a0));
            verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f);
            verts.push_back(0.5f + 0.5f*cosf(a0)); verts.push_back(0.5f + 0.5f*sinf(a0));
            // rim a1
            verts.push_back(r*cosf(a1)); verts.push_back(y); verts.push_back(r*sinf(a1));
            verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f);
            verts.push_back(0.5f + 0.5f*cosf(a1)); verts.push_back(0.5f + 0.5f*sinf(a1));
        }
    }

    // ── BOTTOM CAP ─────────────────────────────────────────────────────────────
    {
        float y = -0.5f, r = getR(0.0f);
        for (int ri = 0; ri < radialSegs; ri++) {
            float a0 = 2.0f * PI * (float)ri       / radialSegs;
            float a1 = 2.0f * PI * (float)(ri + 1) / radialSegs;
            // centre
            verts.push_back(0.0f); verts.push_back(y); verts.push_back(0.0f);
            verts.push_back(0.0f); verts.push_back(-1.0f); verts.push_back(0.0f);
            verts.push_back(0.5f); verts.push_back(0.5f);
            // reversed winding for downward-facing cap
            verts.push_back(r*cosf(a1)); verts.push_back(y); verts.push_back(r*sinf(a1));
            verts.push_back(0.0f); verts.push_back(-1.0f); verts.push_back(0.0f);
            verts.push_back(0.5f + 0.5f*cosf(a1)); verts.push_back(0.5f + 0.5f*sinf(a1));
            verts.push_back(r*cosf(a0)); verts.push_back(y); verts.push_back(r*sinf(a0));
            verts.push_back(0.0f); verts.push_back(-1.0f); verts.push_back(0.0f);
            verts.push_back(0.5f + 0.5f*cosf(a0)); verts.push_back(0.5f + 0.5f*sinf(a0));
        }
    }

    barrelVertexCount = (int)(verts.size() / 8);
    unsigned int vbo;
    glGenVertexArrays(1, &barrelVAO);
    glGenBuffers(1, &vbo);
    glBindVertexArray(barrelVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
}

unsigned int sphereVAO = 0;
int sphereVertexCount = 0;
void buildSphere() {
    std::vector<float> verts;
    int sectorCount = 36;
    int stackCount = 18;
    float radius = 0.5f;
    
    float x, y, z, xy;                           // vertex position
    float nx, ny, nz, lengthInv = 1.0f / radius; // vertex normal
    float s, t;                                  // vertex texCoord
    
    float sectorStep = 2 * glm::pi<float>() / sectorCount;
    float stackStep = glm::pi<float>() / stackCount;
    float sectorAngle, stackAngle;
    
    for(int i = 0; i <= stackCount; ++i) {
        stackAngle = glm::pi<float>() / 2 - i * stackStep;
        xy = radius * cosf(stackAngle);
        y = radius * sinf(stackAngle);
        for(int j = 0; j <= sectorCount; ++j) {
            sectorAngle = j * sectorStep;           
            x = xy * cosf(sectorAngle);             
            z = xy * sinf(sectorAngle);             
            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;
            s = (float)j / sectorCount;
            t = (float)i / stackCount;
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
            verts.push_back(nx); verts.push_back(ny); verts.push_back(nz);
            verts.push_back(s); verts.push_back(t);
        }
    }
    
    std::vector<unsigned int> indices;
    int k1, k2;
    for(int i = 0; i < stackCount; ++i) {
        k1 = i * (sectorCount + 1);
        k2 = k1 + sectorCount + 1;
        for(int j = 0; j < sectorCount; ++j, ++k1, ++k2) {
            if(i != 0) { indices.push_back(k1); indices.push_back(k2); indices.push_back(k1 + 1); }
            if(i != (stackCount-1)) { indices.push_back(k1 + 1); indices.push_back(k2); indices.push_back(k2 + 1); }
        }
    }
    
    sphereVertexCount = indices.size();
    
    std::vector<float> finalVerts;
    for (unsigned int idx : indices) {
        for (int k=0; k<8; k++) finalVerts.push_back(verts[idx*8 + k]);
    }
    
    unsigned int VBO;
    glGenVertexArrays(1, &sphereVAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(sphereVAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, finalVerts.size() * sizeof(float), finalVerts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
}

unsigned int coneVAO = 0;
int coneVertexCount = 0;
void buildCone() {
    std::vector<float> verts;
    int segments = 60;
    float baseRadius = 2.0f;
    float coneHeight = 2.2f;
    
    // Generate cone outline vertices
    for (int i = 0; i <= segments; i++) {
        float angle = 2.0f * glm::pi<float>() * (float)i / (float)segments;
        float x = baseRadius * cosf(angle);
        float z = baseRadius * sinf(angle);
        
        // Base vertex (at floor)
        verts.push_back(x); verts.push_back(0.0f); verts.push_back(z);
        verts.push_back(cosf(angle)); verts.push_back(0.2f); verts.push_back(sinf(angle));
        verts.push_back((float)i / segments); verts.push_back(0.0f);
        
        // Top vertex (apex)
        verts.push_back(0.0f); verts.push_back(coneHeight); verts.push_back(0.0f);
        verts.push_back(cosf(angle)); verts.push_back(0.2f); verts.push_back(sinf(angle));
        verts.push_back((float)i / segments); verts.push_back(1.0f);
    }
    
    coneVertexCount = verts.size() / 8;
    
    unsigned int VBO;
    glGenVertexArrays(1, &coneVAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(coneVAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
}

unsigned int ovalVAO = 0;
int ovalVertexCount = 0;
void buildOval() {
    std::vector<float> verts;
    int segments = 60;
    float radiusX = 4.5f;  // horizontal radius (scaled up)
    float radiusZ = 2.8f;  // vertical radius (depth, scaled up)
    float centerHeight = 0.35f;  // height at center (middle elevated)
    float edgeHeight = 0.0f;     // height at edges (touching floor)
    
    // Generate oval outline vertices with variable height
    for (int i = 0; i <= segments; i++) {
        float angle = 2.0f * glm::pi<float>() * (float)i / (float)segments;
        float x = radiusX * cosf(angle);
        float z = radiusZ * sinf(angle);
        
        // Height varies: center (cos=1) is high, edges (cos=-1) touch floor
        // Use cosine to create smooth dome shape
        float cosAngle = cosf(angle);
        float y = (centerHeight * cosAngle + centerHeight) / 2.0f;
        y = glm::clamp(y, 0.0f, centerHeight);
        
        // Normal pointing up
        float nx = 0.0f;
        float ny = 1.0f;
        float nz = 0.0f;
        
        // Texture coordinates
        float u = (float)i / segments;
        float v = 0.5f;
        
        // Top surface vertex (variable height)
        verts.push_back(x); verts.push_back(y); verts.push_back(z);
        verts.push_back(nx); verts.push_back(ny); verts.push_back(nz);
        verts.push_back(u); verts.push_back(v);
        
        // Bottom surface vertex (at floor level)
        verts.push_back(x); verts.push_back(0.0f); verts.push_back(z);
        verts.push_back(nx); verts.push_back(ny); verts.push_back(nz);
        verts.push_back(u); verts.push_back(v);
    }
    
    // Center point for fan triangulation (at peak height)
    verts.push_back(0.0f); verts.push_back(centerHeight); verts.push_back(0.0f);
    verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f);
    verts.push_back(0.5f); verts.push_back(0.5f);
    
    // Center point at floor level
    verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(0.0f);
    verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f);
    verts.push_back(0.5f); verts.push_back(0.5f);
    
    ovalVertexCount = verts.size() / 8;
    
    unsigned int VBO;
    glGenVertexArrays(1, &ovalVAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(ovalVAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
}


unsigned int mengerVAO = 0;
int mengerVertexCount = 0;

void buildMengerSponge() {
    float baseCube[] = {
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

    std::vector<float> verts;

    auto recurse = [&](auto& self, glm::mat4 model, int depth) -> void {
        if (depth == 0) {
            for (int i = 0; i < 36; i++) {
                glm::vec4 p(baseCube[i * 8], baseCube[i * 8 + 1], baseCube[i * 8 + 2], 1.0f);
                glm::vec4 n(baseCube[i * 8 + 3], baseCube[i * 8 + 4], baseCube[i * 8 + 5], 0.0f);
                
                p = model * p;
                n = glm::normalize(model * n);
                
                verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
                verts.push_back(n.x); verts.push_back(n.y); verts.push_back(n.z);
                verts.push_back(baseCube[i * 8 + 6]); verts.push_back(baseCube[i * 8 + 7]);
            }
            return;
        }

        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                for (int z = -1; z <= 1; z++) {
                    if (abs(x) + abs(y) + abs(z) < 2) continue;
                    glm::mat4 child = glm::translate(model, glm::vec3(x / 3.0f, y / 3.0f, z / 3.0f));
                    child = glm::scale(child, glm::vec3(1.0f / 3.0f));
                    self(self, child, depth - 1);
                }
            }
        }
    };

    recurse(recurse, glm::mat4(1.0f), 3); // Depth 3 => 8000 cubes

    mengerVertexCount = verts.size() / 8;
    unsigned int vbo;
    glGenVertexArrays(1, &mengerVAO);
    glGenBuffers(1, &vbo);
    glBindVertexArray(mengerVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
}

// Massive Central Wall Exhaust Fan utilizing Ruled Surface aerodynamic blades
void drawExhaustFan(Shader& shader) {
    // Shifted to Y=19.0f so the 2x scaled blades do not overlap the ducts at Y=28.0f or floor at Y=0.0f
    glm::vec3 fanCenter(-100.0f, 19.0f, 0.0f);
    
    // Ensure appropriate darkness for heavy industrial metal
    shader.setVec3("objectColor", glm::vec3(0.5f, 0.5f, 0.5f));
    shader.setFloat("material.shininess", 32.0f);
    
    extern unsigned int fanTexture;
    glBindTexture(GL_TEXTURE_2D, fanTexture);
    
    // Central Hub
    extern unsigned int cylinderVAO;
    extern int cylinderVertexCount;
    glBindVertexArray(cylinderVAO);
    glm::mat4 hub = glm::translate(glm::mat4(1.0f), fanCenter);
    // Orient cylinder perfectly along X axis
    hub = glm::rotate(hub, glm::half_pi<float>(), glm::vec3(0, 0, 1));
    hub = glm::scale(hub, glm::vec3(5.0f, 8.0f, 5.0f)); // Scaled 2x to match fan size
    shader.setMat4("model", hub);
    glDrawArrays(GL_TRIANGLES, 0, cylinderVertexCount);
    
    // Blades
    extern unsigned int fanBladeVAO;
    extern int fanBladeVertexCount;
    extern float exhaustFanAngle;
    glBindVertexArray(fanBladeVAO);
    int numBlades = 7;
    
    for (int i = 0; i < numBlades; i++) {
        float bladeAngle = (2.0f * glm::pi<float>() / numBlades) * i - exhaustFanAngle;
        glm::mat4 blade = glm::translate(glm::mat4(1.0f), fanCenter);
        // Spin around X axis
        blade = glm::rotate(blade, bladeAngle, glm::vec3(1, 0, 0));
        blade = glm::scale(blade, glm::vec3(2.0f, 2.0f, 2.0f)); // Scaled 2x
        shader.setMat4("model", blade);
        glDrawArrays(GL_TRIANGLES, 0, fanBladeVertexCount);
    }
}

// Industrial Black Box Paint Chamber -> Arched Tunnel
void drawPaintChamber(Shader& shader, unsigned int cubeVAO, int pathIndex, float centerDist, unsigned int tunnelTex, unsigned int glowTex, unsigned int pipeTex) {
    glm::vec3 pos, dirVec;
    float angle = 0.0f;
    extern void getPathPositionAndAngle(int pathIndex, float dist, glm::vec3& outPos, float& outAngle);
    getPathPositionAndAngle(pathIndex, centerDist, pos, angle);
    
    // New arched curvy tunnel
    extern unsigned int archVAO;
    glBindVertexArray(archVAO);
    glBindTexture(GL_TEXTURE_2D, tunnelTex);
    // Base position
    glm::mat4 tunnel = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0.0f, 0.1f, 0.0f));
    tunnel = glm::rotate(tunnel, angle, glm::vec3(0.0f, 1.0f, 0.0f));
    
    // Ambient glowing neon strips inside Chamber
    shader.setVec3("objectColor", glm::vec3(1.0f, 1.0f, 1.0f));
    shader.setFloat("material.shininess", 16.0f);

    // Scale: Radius=1.5f (width 3.0f to cover 2.9f rails), Length=6.0f
    shader.setMat4("model", glm::scale(tunnel, glm::vec3(1.5f, 1.6f, 6.0f)));
    extern int archVertexCount;
    glDrawArrays(GL_TRIANGLE_STRIP, 0, archVertexCount);

    glBindVertexArray(cubeVAO);

    // Glow pad bottom
    glBindTexture(GL_TEXTURE_2D, glowTex);
    glm::mat4 glow = glm::translate(glm::mat4(1.0f), pos + glm::vec3(0.0f, 0.2f, 0.0f));
    glow = glm::rotate(glow, angle, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 glowScale = glm::vec3(2.0f, 0.4f, 5.8f);
    shader.setMat4("model", glm::scale(glow, glowScale));
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Dynamic Sliding Doors
    glBindTexture(GL_TEXTURE_2D, pipeTex);
    
    for (float zOffset : {-3.0f, 3.0f}) {
        float doorDist = centerDist + zOffset;
        float min_dist = 999.0f;
        for (const GridBox& b : gridBoxes) {
            if (b.state == ON_BELT && b.pathIndex == pathIndex) {
                float dist = std::abs(b.distance - doorDist);
                if (dist < min_dist) min_dist = dist;
            }
        }

        // Open the door totally if box is closer than 1.0, start opening at 2.2
        float openRatio = 0.0f;
        if (min_dist < 2.2f) {
            openRatio = glm::clamp((2.2f - min_dist) / 1.2f, 0.0f, 1.0f);
        }

        glm::vec3 doorLocalEnd(0.0f, 0.9f, zOffset);
        glm::mat4 localMat = glm::translate(glm::mat4(1.0f), pos);
        localMat = glm::rotate(localMat, angle, glm::vec3(0,1,0));
        glm::vec3 curPos = glm::vec3(localMat * glm::vec4(doorLocalEnd, 1.0f));

        // 1. Draw Door Housings (Pillars)
        glBindTexture(GL_TEXTURE_2D, pipeTex);
        for (float sideSign : {-1.0f, 1.0f}) {
            glm::mat4 pillar = glm::translate(glm::mat4(1.0f), curPos);
            pillar = glm::rotate(pillar, angle, glm::vec3(0,1,0));
            pillar = glm::translate(pillar, glm::vec3(sideSign * 1.05f, 0, 0));
            glm::vec3 pScale = glm::vec3(0.8f, 1.6f, 0.12f);
            shader.setMat4("model", glm::scale(pillar, pScale));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        // 2. Draw Top Header to connect pillars
        {
            glm::vec3 headerPos = curPos;
            headerPos.y += 0.65f;
            glm::mat4 header = glm::translate(glm::mat4(1.0f), headerPos);
            header = glm::rotate(header, angle, glm::vec3(0,1,0));
            glm::vec3 hScale = glm::vec3(2.9f, 0.3f, 0.12f);
            shader.setMat4("model", glm::scale(header, hScale));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        // 3. Draw sliding doors behind the housings
        glBindTexture(GL_TEXTURE_2D, pipeTex);
        for (float sideSign : {-1.0f, 1.0f}) {
            float closeOffset = 0.33f;
            float slideDist = openRatio * 0.65f;

            glm::mat4 doorMat = glm::translate(glm::mat4(1.0f), curPos);
            doorMat = glm::rotate(doorMat, angle, glm::vec3(0,1,0));
            doorMat = glm::translate(doorMat, glm::vec3(sideSign * (closeOffset + slideDist), 0, (zOffset > 0 ? -0.05f : 0.05f)));

            glm::vec3 dScale = glm::vec3(0.65f, 1.3f, 0.04f);
            shader.setMat4("model", glm::scale(doorMat, dScale));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }
}

// Rotating Box-Cleaning Station (using Procedural Cylinders)
void drawCleaningStation(Shader& shader, unsigned int cubeVAO, int pathIndex, float centerDist, unsigned int cylinderTex, unsigned int mountTex) {
    glm::vec3 pos;
    float angle = 0.0f;
    extern void getPathPositionAndAngle(int pathIndex, float dist, glm::vec3& outPos, float& outAngle);
    getPathPositionAndAngle(pathIndex, centerDist, pos, angle);

    extern unsigned int cylinderVAO;
    extern int cylinderVertexCount;
    float time = (float)glfwGetTime();

    // 1. Draw base mounts holding the rollers (left and right sides of belt)
    glBindVertexArray(cubeVAO);
    glBindTexture(GL_TEXTURE_2D, mountTex);
    
    for (float side : {-1.5f, 1.5f}) {
        glm::mat4 mountMat = glm::translate(glm::mat4(1.0f), pos);
        mountMat = glm::rotate(mountMat, angle, glm::vec3(0,1,0));
        mountMat = glm::translate(mountMat, glm::vec3(side, 0.4f, 0.0f));
        shader.setMat4("model", glm::scale(mountMat, glm::vec3(0.2f, 1.0f, 1.0f)));
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }

    // 2. Draw 3 spinning brush cylinders spanning across the belt
    glBindVertexArray(cylinderVAO);
    glBindTexture(GL_TEXTURE_2D, cylinderTex);
    
    for (int i = 0; i < 3; i++) {
        float zOffset = -0.4f + i * 0.4f;
        glm::mat4 cylMat = glm::translate(glm::mat4(1.0f), pos);
        cylMat = glm::rotate(cylMat, angle, glm::vec3(0,1,0));
        // Hover at Y=0.7 (just touching top of boxes)
        cylMat = glm::translate(cylMat, glm::vec3(0.0f, 0.7f, zOffset));
        
        // Spin rapidly (rotate around local X-axis)
        float spinSpeed = 10.0f;
        cylMat = glm::rotate(cylMat, time * spinSpeed + i, glm::vec3(1,0,0));
        // Rotate 90 deg around Y to span horizontally
        cylMat = glm::rotate(cylMat, glm::radians(90.0f), glm::vec3(0,1,0));
        // Scale to reach both sides (length = 3.0)
        shader.setMat4("model", glm::scale(cylMat, glm::vec3(0.4f, 3.0f, 0.4f)));
        glDrawArrays(GL_TRIANGLES, 0, cylinderVertexCount);
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
void renderScene(Shader& shader, unsigned int VAO, unsigned int boxTex, unsigned int conveyorTex, unsigned int floorTex, unsigned int wallTex, unsigned int blueTex, unsigned int tunnelTex, unsigned int walkTexture, unsigned int whiteLightTex, unsigned int bakaTexture, unsigned int coneTexture, glm::mat4 view, glm::mat4 projection, glm::vec3 camPos);

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
    // glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

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

    buildHelicoidSlide();
    buildCatwalkSystem();
    buildDuctworkSystem();
    buildArch();
    buildFanBlade();
    buildCylinder();
    buildSphere();
    buildOval();
    buildCone();
    buildBarrel();
    buildMengerSponge();

    std::string boxTexPath = getResourcePath("texture", "box.jpg");
    std::string conveyorTexPath = getResourcePath("texture", "conveyor.jpg");
    std::string floorTexPath = getResourcePath("texture", "floor.jpg");
    std::string wallTexPath = getResourcePath("texture", "wall.jpg");
    std::string blueTexPath = getResourcePath("texture", "blue.jpg");
    std::string tunnelTexPath = getResourcePath("texture", "tunnel.png");
    std::string randomPropTexPath = getResourcePath("texture", "randomProp.jpg");
    std::string barrelTexPath = getResourcePath("texture", "barrel.jpg");
    std::string doorTexPath = getResourcePath("texture", "door.jpg");
    std::string bakaTexPath = getResourcePath("texture", "baka.jpg");
    std::string coneTexPath = getResourcePath("texture", "cone.jpg");
    std::string walkTexPath = "walk.jpg";
    std::string ductTexPath = "duct.jpg";
    std::string fanTexPath  = "fan.jpg";
    std::string watchTexPath = "watch.jpg";

    unsigned int boxTexture = loadTexture(boxTexPath.c_str(), 160, 100, 50); 
    unsigned int walkTexture = loadTexture(walkTexPath.c_str(), 200, 200, 200); 
    unsigned int conveyorTexture = loadTexture(conveyorTexPath.c_str(), 30, 30, 30); 
    unsigned int floorTexture = loadTexture(floorTexPath.c_str(), 180, 180, 180); 
    unsigned int wallTexture = loadTexture(wallTexPath.c_str(), 120, 140, 160); 
    unsigned int blueTexture = loadTexture(blueTexPath.c_str(), 0, 150, 255); 
    unsigned int tunnelTexture = loadTexture(tunnelTexPath.c_str(), 200, 200, 200); 
    unsigned int whiteLightTexture = loadTexture("", 255, 255, 255); 
    unsigned int randomPropTexture = loadTexture(randomPropTexPath.c_str(), 200, 100, 50);
    barrelTexture = loadTexture(barrelTexPath.c_str(), 80, 80, 200);
    unsigned int doorTexture = loadTexture(doorTexPath.c_str(), 100, 100, 100);
    unsigned int bakaTexture = loadTexture(bakaTexPath.c_str(), 139, 69, 19);
    unsigned int coneTexture = loadTexture(coneTexPath.c_str(), 200, 128, 64);
    ductTexture  = loadTexture(ductTexPath.c_str(),  150, 150, 150);
    fanTexture   = loadTexture(fanTexPath.c_str(),   100, 100, 100);
    watchTexture = loadTexture(watchTexPath.c_str(), 180, 150, 120);
    
    std::string fracTexPath = "frac.jpg";
    fracTexture = loadTexture(fracTexPath.c_str(), 150, 150, 150);

    std::string streetTexPath = getResourcePath("texture", "street.jpg");
    std::string grassTexPath  = getResourcePath("texture", "grass.jpg");
    std::string skyTexPath    = getResourcePath("texture", "sky.jpg");
    streetTexture = loadTexture(streetTexPath.c_str(), 80,  80,  85);
    grassTexture  = loadTexture(grassTexPath.c_str(),  50, 160,  60);
    skyTexture    = loadTexture(skyTexPath.c_str(),   135, 206, 235);

    // Define 5 parallel belts — alternating curve direction, R=40, bz from -40 to +40
    for (int b = 0; b < 5; b++) {
        globalPaths[b].clear();
        float bz = -40.0f + b * 20.0f; // -40, -20, 0, 20, 40

        // Alternate: even belts curve toward +Z, odd belts curve toward -Z
        // to keep all arcs inside the ±100 room boundary
        float R     = 40.0f;
        float sweep = (b % 2 == 0) ? -glm::pi<float>() : glm::pi<float>();
        float beltY = (b % 2 == 0) ? 0.8f : 4.5f; // even=lower, odd=upper

        // Short straight lead-in from arm to arc start
        globalPaths[b].push_back({STRAIGHT, 6.0f,
            glm::vec3(-46.0f, beltY, bz), glm::vec3(-R, beltY, bz),
            glm::vec3(), 0, 0, 0});
        // Large semicircle
        globalPaths[b].push_back({CURVE, R * glm::pi<float>(),
            glm::vec3(-R, beltY, bz), glm::vec3(R, beltY, bz),
            glm::vec3(0.0f, beltY, bz), R, glm::pi<float>(), sweep});
        // Short straight lead-out to arm
        globalPaths[b].push_back({STRAIGHT, 6.0f,
            glm::vec3(R, beltY, bz), glm::vec3(46.0f, beltY, bz),
            glm::vec3(), 0, 0, 0});
    }

    // Initialize Shelves — 10 towers, 2 per belt, offset ±3 in Z so they don't overlap
    for(int side = 0; side < 2; side++) {
        for(int t = 0; t < SHELF_TOWERS; t++) {
            // bz = belt center, then +/- 3 for even/odd tower
            float beltZ  = -40.0f + (t / 2) * 20.0f;
            float zOff   = (t % 2 == 0) ? -3.0f : 3.0f;
            float zPos   = beltZ + zOff;

            for(int tier = 0; tier < SHELF_TIERS; tier++) {
                for(int slot = 0; slot < SHELF_SLOTS; slot++) {
                    shelfOccupied[side][t][tier][slot] = (side == 0);
                    float xPos = (side == 0) ? -52.0f : 52.0f;
                    // Adjusted bx slightly inward to avoid protruding over shelf bounds
                    float bx   = (side == 0) ?  0.8f  : -0.8f;
                    // Adjusted bz to evenly space 9 boxes across the 9.0 length shelf
                    float bz   = -4.0f + slot * 1.0f;
                    shelfSlotPos[side][t][tier][slot] =
                        glm::vec3(xPos + bx, tier * 3.0f + 0.6f, zPos + bz);
                }
            }
        }
    }

    // Initialize the 20 shelf arms — each tower at its own (non-overlapping) Z
    for (int t = 0; t < SHELF_TOWERS; t++) {
        float beltZ = -40.0f + (t / 2) * 20.0f;
        float zOff  = (t % 2 == 0) ? -3.0f : 3.0f;
        float zPos  = beltZ + zOff;

        // Left Arms (Source) — park just outside belt start X=-46
        shelfArms[t].basePos    = glm::vec3(-48.0f, 0.0f, zPos);
        shelfArms[t].baseRotY   = 90.0f;
        shelfArms[t].towerIndex = t;

        // Right Arms (Dest) — park just outside belt end X=+46
        shelfArms[10 + t].basePos    = glm::vec3(48.0f, 0.0f, zPos);
        shelfArms[10 + t].baseRotY   = -90.0f;
        shelfArms[10 + t].towerIndex = t;
    }

    gridBoxes.clear();


    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        // Update animations
        extern float exhaustFanSpeed;
        extern float exhaustFanAngle;
        exhaustFanAngle += exhaustFanSpeed * deltaTime;
        
        extern float conveyorTexScroll, conveyorSpeed;
        conveyorTexScroll += conveyorSpeed * deltaTime;
        
        if (fanOn) fanAngle += 150.0f * deltaTime;
        robotBaseAngle += 20.0f * deltaTime;
        robotElbowAngle = sin(glfwGetTime() * 2.0f) * 30.0f;
        // Gripper cycles: open(0.7) -> close(0.1) -> hold -> repeat
        gripperSpread = 0.4f + 0.35f * sin(glfwGetTime() * 2.5f); // oscillates 0.05..0.75

        // Particle Physics
        for (int i = 0; i < (int)sparkParticles.size(); ) {
            sparkParticles[i].position += sparkParticles[i].velocity * deltaTime;
            sparkParticles[i].velocity.y -= 9.8f * deltaTime; // gravity
            sparkParticles[i].life -= deltaTime;
            if (sparkParticles[i].life <= 0.0f) {
                sparkParticles.erase(sparkParticles.begin() + i);
            } else {
                i++;
            }
        }

        // Door Animation (smoothly slides up until offset is 15.0f, down to 0.0f)
        float doorSpeed = 8.0f;
        if (doorOpen) {
            doorYOffset += doorSpeed * deltaTime;
            if (doorYOffset > 15.0f) doorYOffset = 15.0f;
        } else {
            doorYOffset -= doorSpeed * deltaTime;
            if (doorYOffset < 0.0f) doorYOffset = 0.0f;
        }

        // --- BOX PROCESSING STAGE MACHINE ---
        // conveyorSpeed is a global, controlled by + / - keys in processInput()
        float startClearDist = 6.0f;

        // --- BOX STATE MACHINE ---
        std::vector<GridBox> newBoxes;

        for (int i = 0; i < (int)gridBoxes.size(); i++) {
            GridBox& b = gridBoxes[i];

            if (b.state == ON_BELT) {
                float proposedDist = b.distance + conveyorSpeed * deltaTime;
                float beltLen = getPathTotalLength(b.pathIndex);
                float exitDist = beltLen - 1.5f;

                // Basic Box Collision Detection
                float maxAllowedDist = 9999.0f; 
                for (int j = 0; j < (int)gridBoxes.size(); j++) {
                    if (i == j) continue;
                    const GridBox& ob = gridBoxes[j];
                    if (ob.pathIndex == b.pathIndex && (ob.state == ON_BELT || ob.state == WAITING_FOR_PICKUP)) {
                        float distOb = ob.distance;
                        if (ob.state == WAITING_FOR_PICKUP) distOb = exitDist;
                        if (distOb > b.distance) {
                            float allowed = distOb - 2.5f; // Box centers must remain 2.5 units apart
                            if (allowed < maxAllowedDist) maxAllowedDist = allowed;
                        }
                    }
                }

                if (proposedDist > maxAllowedDist) {
                    proposedDist = maxAllowedDist;
                }

                if (proposedDist > b.distance) {
                    b.distance = proposedDist;
                }

                if (b.distance > beltLen * 0.5f && b.stage == RAW) {
                    b.stage = PAINTED; // Paint at halfway mark
                }

                if (b.distance >= exitDist) {
                    // Wait at end for right arm
                    glm::vec3 endPos; float tempAng;
                    getPathPositionAndAngle(b.pathIndex, exitDist, endPos, tempAng);
                    b.worldPos = endPos;
                    b.worldPos.y += 0.6f;
                    b.state = WAITING_FOR_PICKUP;
                }
            }
            else if (b.state == BEING_PICKED) {
                // Picked by arm
                b.pickTimer += deltaTime;
                if (b.pickTimer > 8.0f) {
                    // Safety timeout drop
                    b.state = ON_SHELF;
                }
            }
        }

        // ----- SHELF ARM AI -----
        for (int a = 0; a < 20; a++) {
            ShelfArm& arm = shelfArms[a];
            int side = (a < 10) ? 0 : 1; // 0 = src/left, 1 = dest/right
            int tIdx = arm.towerIndex;
            int pathIndex = tIdx / 2; // 0..9 maps to belts 0..4

            if (!arm.armBusy) {
                if (side == 0) {
                    // SOURCE ARM: If belt start is clear for this specific belt
                    bool thisBeltClear = true;

                    // 1. Check no box is ON_BELT near the start
                    for (const auto& b : gridBoxes) {
                        if (b.pathIndex == pathIndex && b.distance < startClearDist &&
                            (b.state == ON_BELT || b.state == BEING_PICKED)) {
                            thisBeltClear = false; break;
                        }
                    }

                    // 2. Check that no sibling source arm for the SAME belt is already busy
                    //    (prevents two arms placing simultaneously in the same frame)
                    if (thisBeltClear) {
                        for (int oa = 0; oa < 10; oa++) {
                            if (oa != a && shelfArms[oa].armBusy &&
                                (shelfArms[oa].towerIndex / 2) == pathIndex) {
                                thisBeltClear = false; break;
                            }
                        }
                    }

                    if (thisBeltClear) {
                        for (int t = 0; t < SHELF_TIERS; t++) {
                            for (int s = 0; s < SHELF_SLOTS; s++) {
                                if (shelfOccupied[side][tIdx][t][s]) {
                                    shelfOccupied[side][tIdx][t][s] = false;
                                    
                                    GridBox nb;
                                    nb.distance  = 0.0f;
                                    nb.stage     = RAW;
                                    nb.state     = BEING_PICKED;
                                    nb.worldPos  = shelfSlotPos[side][tIdx][t][s];
                                    nb.pathIndex = pathIndex;
                                    gridBoxes.push_back(nb);
                                    
                                    arm.pickBoxIndex = (int)gridBoxes.size() - 1;
                                    arm.pickupPos    = shelfSlotPos[side][tIdx][t][s];
                                    
                                    glm::vec3 startPos; float junkAng;
                                    getPathPositionAndAngle(pathIndex, 0.0f, startPos, junkAng);
                                    arm.placePos = startPos + glm::vec3(0, 0.6f, 0);
                                    
                                    arm.armBusy    = true;
                                    arm.phase      = 0.0f;
                                    arm.phaseTimer = 0.0f;
                                    arm.carrying   = false;
                                    arm.effectorPos = arm.basePos + glm::vec3(0, 10.0f, 0);
                                    goto nextArm;
                                }
                            }
                        }
                    }
                } else {
                    // DEST ARM: Look for box waiting at end of belt this arm is assigned to
                    int closest = -1;
                    for (int i = 0; i < (int)gridBoxes.size(); i++) {
                        if (gridBoxes[i].state == WAITING_FOR_PICKUP && gridBoxes[i].pathIndex == pathIndex) {
                            closest = i; break;
                        }
                    }
                    if (closest >= 0) {
                        for (int t = 0; t < SHELF_TIERS; t++) {
                            for (int s = 0; s < SHELF_SLOTS; s++) {
                                if (!shelfOccupied[side][tIdx][t][s]) {
                                    shelfOccupied[side][tIdx][t][s] = true;
                                    arm.pickBoxIndex = closest;
                                    arm.pickupPos = gridBoxes[closest].worldPos;
                                    arm.placePos = shelfSlotPos[side][tIdx][t][s];
                                    gridBoxes[closest].state = BEING_PICKED;
                                    gridBoxes[closest].pickTimer = 0.0f;
                                    arm.armBusy = true;
                                    arm.phase = 0.0f;
                                    arm.phaseTimer = 0.0f;
                                    arm.carrying = false;
                                    arm.effectorPos = arm.basePos + glm::vec3(0, 10.0f, 0);
                                    goto nextArm;
                                }
                            }
                        }
                    }
                }
            } else {
                arm.phaseTimer += deltaTime;
                glm::vec3 idlePos = arm.basePos + glm::vec3(0, 10.0f, 0);

                if (arm.phase == 0.0f) {
                    float t = glm::clamp(arm.phaseTimer / 0.4f, 0.0f, 1.0f);
                    t = t * t * (3.0f - 2.0f * t);
                    arm.effectorPos = glm::mix(idlePos, arm.pickupPos, t);
                    if (arm.phaseTimer >= 0.4f) {
                        arm.phase = 1.0f; arm.phaseTimer = 0.0f;
                        arm.carrying = true;
                        spawnSparks(arm.effectorPos, 20); // SPARK TRIGGER
                    }
                }
                else if (arm.phase == 1.0f) {
                    float t = glm::clamp(arm.phaseTimer / 0.3f, 0.0f, 1.0f);
                    t = t * t * (3.0f - 2.0f * t);
                    glm::vec3 liftPos = arm.pickupPos + glm::vec3(0, 4.0f, 0);
                    arm.effectorPos = glm::mix(arm.pickupPos, liftPos, t);
                    if (arm.phaseTimer >= 0.3f) { arm.phase = 2.0f; arm.phaseTimer = 0.0f; }
                }
                else if (arm.phase == 2.0f) {
                    float t = glm::clamp(arm.phaseTimer / 0.6f, 0.0f, 1.0f);
                    t = t * t * (3.0f - 2.0f * t);
                    glm::vec3 liftPos = arm.pickupPos + glm::vec3(0, 4.0f, 0);
                    glm::vec3 dropPos = arm.placePos + glm::vec3(0, 4.0f, 0);

                    if (t < 0.7f) arm.effectorPos = glm::mix(liftPos, dropPos, t / 0.7f);
                    else arm.effectorPos = glm::mix(dropPos, arm.placePos, (t - 0.7f) / 0.3f);

                    if (arm.phaseTimer >= 0.6f) {
                        if (arm.pickBoxIndex >= 0 && arm.pickBoxIndex < (int)gridBoxes.size()) {
                            gridBoxes[arm.pickBoxIndex].worldPos = arm.placePos;
                            gridBoxes[arm.pickBoxIndex].state = (side == 1) ? ON_SHELF : ON_BELT;
                            if (side == 1) gridBoxes[arm.pickBoxIndex].stage = BOUND;
                        }
                        arm.armBusy = false;
                        arm.carrying = false;
                        arm.pickBoxIndex = -1;
                        arm.phase = 0.0f;
                        arm.phaseTimer = 0.0f;
                    }
                }

                if (arm.carrying && arm.pickBoxIndex >= 0 && arm.pickBoxIndex < (int)gridBoxes.size()) {
                    gridBoxes[arm.pickBoxIndex].worldPos = arm.effectorPos;
                }
            }

            if (!arm.armBusy) {
                arm.effectorPos = glm::mix(arm.effectorPos, arm.basePos + glm::vec3(0, 10.0f, 0), deltaTime * 2.0f);
            }
        nextArm:;
        }

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Follow a random active box
        if (!gridBoxes.empty()) {
            glm::vec3 tgtPos; float junkAng;
            if (gridBoxes[0].state == ON_BELT) {
                getPathPositionAndAngle(gridBoxes[0].pathIndex, gridBoxes[0].distance, tgtPos, junkAng);
            } else {
                tgtPos = gridBoxes[0].worldPos;
            }
            followCamera.SetTarget(tgtPos);
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        float aspect = singleViewport ? (float)width / (float)height : (float)(width / 2) / (float)(height / 2);
        glm::mat4 projection = customPerspective(glm::radians(45.0f), aspect, 0.1f, 1000.0f);

        if (singleViewport) {
            glViewport(0, 0, width, height);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, walkTexture, whiteLightTexture, bakaTexture, coneTexture, mainCamera.GetViewMatrix(), projection, mainCamera.Position);
        }
        else {
            // 1. Top-Left Viewport (Main Interactive Camera)
            glViewport(0, height / 2, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, walkTexture, whiteLightTexture, bakaTexture, coneTexture, mainCamera.GetViewMatrix(), projection, mainCamera.Position);

            // 2. Top-Right Viewport (Bird's Eye)
            glViewport(width / 2, height / 2, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, walkTexture, whiteLightTexture, bakaTexture, coneTexture, birdEyeCamera.GetViewMatrix(), projection, birdEyeCamera.Position);

            // 3. Bottom-Left Viewport (Follow Camera)
            glViewport(0, 0, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, walkTexture, whiteLightTexture, bakaTexture, coneTexture, followCamera.GetViewMatrix(), projection, followCamera.Position);

            // 4. Bottom-Right Viewport (Static Front Camera)
            glViewport(width / 2, 0, width / 2, height / 2);
            renderScene(usePhong ? phongShader : gouraudShader, VAO, boxTexture, conveyorTexture, floorTexture, wallTexture, blueTexture, tunnelTexture, walkTexture, whiteLightTexture, bakaTexture, coneTexture, frontCamera.GetViewMatrix(), projection, frontCamera.Position);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glfwTerminate();
    return 0;
}



void renderScene(Shader& shader, unsigned int VAO, unsigned int boxTex, unsigned int conveyorTex, unsigned int floorTex, unsigned int wallTex, unsigned int blueTex, unsigned int tunnelTex, unsigned int walkTexture, unsigned int whiteLightTex, unsigned int bakaTexture, unsigned int coneTexture, glm::mat4 view, glm::mat4 projection, glm::vec3 camPos) {
    shader.use();
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);

    // Pass texture fallback info
    shader.setBool("useTextureColorOnly", useTextureColorOnly);
    shader.setBool("textureOn", textureOn);

    // Explicitly set material uniforms
    shader.setInt("material.diffuse", 0);
    shader.setInt("material.specular", 0); // use diffuse map for specular as well for simplicity
    shader.setFloat("material.shininess", 32.0f);

    bool effLight = masterLightOn || p_override_light;
    shader.setBool("ambientOn", effLight && ambientOn);
    shader.setBool("diffuseOn", effLight && diffuseOn);
    shader.setBool("specularOn", effLight && specularOn);
    shader.setBool("dirLightOn", masterLightOn && dirLightOn);
    shader.setBool("pointLightOn", effLight && pointLightOn);
    shader.setBool("spotLightOn", effLight && spotLightOn);

    // Setup Directional Light (overhead sky — moderate, not blinding)
    shader.setVec3("dirLight.direction", 0.0f, -1.0f, 0.0f);
    shader.setVec3("dirLight.ambient",  0.35f, 0.35f, 0.35f); // base shadow fill
    shader.setVec3("dirLight.diffuse",  0.55f, 0.55f, 0.55f); // main surface light
    shader.setVec3("dirLight.specular", 0.2f,  0.2f,  0.2f);

    // 4 corner ceiling point lights — industrial warm-white
    // low attenuation so they reach far, but ambient kept low to avoid washout
    shader.setVec3("pointLights[0].position", -60.0f, 14.0f, 60.0f);
    shader.setVec3("pointLights[0].ambient",  0.12f, 0.12f, 0.13f);
    shader.setVec3("pointLights[0].diffuse",  0.8f,  0.8f,  0.75f);
    shader.setVec3("pointLights[0].specular", 0.4f,  0.4f,  0.4f);
    shader.setFloat("pointLights[0].constant",  1.0f);
    shader.setFloat("pointLights[0].linear",    0.009f);
    shader.setFloat("pointLights[0].quadratic", 0.0003f);

    shader.setVec3("pointLights[1].position", 60.0f, 14.0f, 60.0f);
    shader.setVec3("pointLights[1].ambient",  0.12f, 0.12f, 0.13f);
    shader.setVec3("pointLights[1].diffuse",  0.8f,  0.8f,  0.75f);
    shader.setVec3("pointLights[1].specular", 0.4f,  0.4f,  0.4f);
    shader.setFloat("pointLights[1].constant",  1.0f);
    shader.setFloat("pointLights[1].linear",    0.009f);
    shader.setFloat("pointLights[1].quadratic", 0.0003f);

    shader.setVec3("pointLights[2].position", -60.0f, 14.0f, -60.0f);
    shader.setVec3("pointLights[2].ambient",  0.12f, 0.12f, 0.13f);
    shader.setVec3("pointLights[2].diffuse",  0.8f,  0.8f,  0.75f);
    shader.setVec3("pointLights[2].specular", 0.4f,  0.4f,  0.4f);
    shader.setFloat("pointLights[2].constant",  1.0f);
    shader.setFloat("pointLights[2].linear",    0.009f);
    shader.setFloat("pointLights[2].quadratic", 0.0003f);

    shader.setVec3("pointLights[3].position", 60.0f, 14.0f, -60.0f);
    shader.setVec3("pointLights[3].ambient",  0.12f, 0.12f, 0.13f);
    shader.setVec3("pointLights[3].diffuse",  0.8f,  0.8f,  0.75f);
    shader.setVec3("pointLights[3].specular", 0.4f,  0.4f,  0.4f);
    shader.setFloat("pointLights[3].constant",  1.0f);
    shader.setFloat("pointLights[3].linear",    0.009f);
    shader.setFloat("pointLights[3].quadratic", 0.0003f);

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
    glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
    extern std::vector<PathSegment> globalPaths[5];
    extern float conveyorTexScroll;
    for (int b = 0; b < 5; b++) {
    float distanceAlongBelt = 0.0f;
    for (size_t i = 0; i < globalPaths[b].size(); i++) {
        const auto& seg = globalPaths[b][i];
        if (seg.type == STRAIGHT) {
            glm::vec3 dir = seg.end - seg.start;
            float len = glm::length(dir);
            glm::vec3 center = seg.start + dir * 0.5f;
            float angle = atan2(dir.x, dir.z);

            glm::mat4 model = glm::translate(glm::mat4(1.0f), center);
            model = glm::rotate(model, angle, glm::vec3(0, 1, 0));
            
            // ★ MOVING BELT SURFACE — scrolling slats ★
            // Side rails removed for open start/end

            // Flat rigid bed to connect with the rest of the belt curve
            glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
            glm::mat4 bedM = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f));
            shader.setMat4("model", glm::scale(bedM, glm::vec3(2.8f, 0.2f, len)));
            
            float beltSpdMult = (i == 0 || i == globalPaths[b].size() - 1) ? 0.0f : 1.0f;
            shader.setBool("useTexOffset", true);
            shader.setVec2("texScale", glm::vec2(1.0f, len * 0.5f));
            shader.setVec2("texOffset", glm::vec2(0.0f, -distanceAlongBelt * 0.5f + conveyorTexScroll * beltSpdMult * 0.5f));
            
            glDrawArrays(GL_TRIANGLES, 0, 36);
            
            shader.setBool("useTexOffset", false);
            shader.setVec2("texScale", glm::vec2(1.0f, 1.0f));

            // Animated slats — thin boxes sliding along belt direction
            glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
            {
                float beltSpeed  = (i == 0 || i == globalPaths[b].size() - 1) ? 0.0f : 3.0f;        // units / sec
                float slatSpacing = 0.55f;       // gap between slat centres
                float slatThick  = 0.42f;        // slat length along belt
                float slatH      = 0.13f;        // slat height (slightly taller to sit on the flat bed)
                float slatW      = 2.6f;         // spanning belt width

                // Continuous unwrapped scroll - advances every frame, no wrapping
                float timeScroll = (float)glfwGetTime() * beltSpeed;
                
                // Apply modulo only for the pattern repetition, not the motion
                float scrollPattern = fmod(timeScroll, slatSpacing);

                // Calculate range of slat indices needed to cover the entire segment
                int startSlat = (int)floor(-(len * 0.5f + slatSpacing) / slatSpacing) - 1;
                int endSlat = (int)ceil((len * 0.5f + slatSpacing) / slatSpacing) + 1;

                // Draw slats filling the segment with continuous motion
                for (int sl = startSlat; sl <= endSlat; sl++) {
                    // Position: base slat spacing minus continuous scroll offset
                    float d = sl * slatSpacing - scrollPattern - len * 0.5f;

                    glm::mat4 slatM = glm::translate(model,
                        glm::vec3(0.0f, 0.07f, d)); // centered on belt now
                    slatM = glm::scale(slatM,
                        glm::vec3(slatW, slatH, slatThick));
                    shader.setMat4("model", slatM);
                    glDrawArrays(GL_TRIANGLES, 0, 36);
                }
            }

            // Legs
            glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
            int numLegs = (int)(len / 4.0f);
            for (int j = 0; j <= numLegs; j++) {
                float dist = j * 4.0f - len / 2.0f;
                glm::mat4 legCenter = glm::translate(model, glm::vec3(0.0f, 0.0f, dist));
                glm::mat4 legL = glm::translate(legCenter, glm::vec3(-1.3f, -0.45f, 0.0f));
                shader.setMat4("model", glm::scale(legL, glm::vec3(0.2f, 0.9f, 0.2f)));
                glDrawArrays(GL_TRIANGLES, 0, 36);
                glm::mat4 legR = glm::translate(legCenter, glm::vec3(1.3f, -0.45f, 0.0f));
                shader.setMat4("model", glm::scale(legR, glm::vec3(0.2f, 0.9f, 0.2f)));
                glDrawArrays(GL_TRIANGLES, 0, 36);
            }

            // Side guide rails removed for open start/end

            // Spinning Rollers removed completely
        } else {
            // CURVE: render as many small boxes approximating the curve with scrolling animation
            glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
            
            // Belt scrolling for curves
            float beltSpeed  = 3.0f;
            float slatSpacing = 0.55f;
            float timeScroll = (float)glfwGetTime() * beltSpeed;
            float scrollPattern = fmod(timeScroll, slatSpacing);
            
            // Calculate total arc length for scrolling
            float arcLength = seg.length;
            int numSlatPositions = (int)(arcLength / slatSpacing) + 3;
            
            int segments = 60; // more segments = smoother arc
            for(int s=0; s<segments; s++) {
                float t1 = (float)s / segments;
                float t2 = (float)(s+1) / segments;
                float a1 = seg.startAngle + t1 * seg.sweepAngle;
                float a2 = seg.startAngle + t2 * seg.sweepAngle;
                glm::vec3 p1 = seg.center + glm::vec3(cos(a1)*seg.radius, seg.start.y - seg.center.y, sin(a1)*seg.radius);
                glm::vec3 p2 = seg.center + glm::vec3(cos(a2)*seg.radius, seg.start.y - seg.center.y, sin(a2)*seg.radius);
                
                glm::vec3 dir = p2 - p1;
                float len = glm::length(dir);
                glm::vec3 center = p1 + dir * 0.5f;
                float angle = atan2(dir.x, dir.z);

                glm::mat4 model = glm::translate(glm::mat4(1.0f), center);
                model = glm::rotate(model, angle, glm::vec3(0, 1, 0));
                
                glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
                shader.setMat4("model", glm::scale(model, glm::vec3(2.8f, 0.2f, len)));
                
                shader.setBool("useTexOffset", true);
                shader.setVec2("texScale", glm::vec2(1.0f, len * 0.5f));
                float currentCurveDist = t1 * arcLength;
                shader.setVec2("texOffset", glm::vec2(0.0f, -(distanceAlongBelt + currentCurveDist) * 0.5f + conveyorTexScroll * 0.5f));

                glDrawArrays(GL_TRIANGLES, 0, 36);
                
                shader.setBool("useTexOffset", false);
                shader.setVec2("texScale", glm::vec2(1.0f, 1.0f));

                if (s % 5 == 0) {
                    glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
                    glm::mat4 legL = glm::translate(model, glm::vec3(-1.3f, -0.45f, 0.0f));
                    shader.setMat4("model", glm::scale(legL, glm::vec3(0.2f, 0.9f, 0.2f)));
                    glDrawArrays(GL_TRIANGLES, 0, 36);
                    glm::mat4 legR = glm::translate(model, glm::vec3(1.3f, -0.45f, 0.0f));
                    shader.setMat4("model", glm::scale(legR, glm::vec3(0.2f, 0.9f, 0.2f)));
                    glDrawArrays(GL_TRIANGLES, 0, 36);
                }
            }
            
            // Draw animated slats on the curve
            glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
            for (int sl = 0; sl < numSlatPositions; sl++) {
                // Position along arc (distance parameter)
                float distAlongArc = sl * slatSpacing - scrollPattern;
                
                // Skip if completely outside the arc
                if (distAlongArc < -slatSpacing || distAlongArc > arcLength + slatSpacing) continue;
                
                // Clamp distance to valid arc range
                float clampedDist = glm::clamp(distAlongArc, 0.0f, arcLength);
                
                // Normalize to [0, 1] for interpolation
                float t = clampedDist / arcLength;
                
                // Get position and angle on the curve
                float currentAngle = seg.startAngle + t * seg.sweepAngle;
                glm::vec3 slatPos = seg.center + glm::vec3(
                    cos(currentAngle) * seg.radius, 
                    seg.start.y - seg.center.y, 
                    sin(currentAngle) * seg.radius
                );
                
                // Tangent direction along curve
                float tangentAngle = currentAngle + (seg.sweepAngle > 0 ? glm::pi<float>()/2.0f : -glm::pi<float>()/2.0f);
                float slatAngle = atan2(sin(tangentAngle), cos(tangentAngle));
                
                glm::mat4 slatM = glm::translate(glm::mat4(1.0f), slatPos);
                slatM = glm::rotate(slatM, slatAngle, glm::vec3(0, 1, 0));
                slatM = glm::scale(slatM, glm::vec3(2.6f, 0.12f, 0.42f));
                shader.setMat4("model", slatM);
                glDrawArrays(GL_TRIANGLES, 0, 36);
            }
        }
        distanceAlongBelt += seg.length;
    }
    }

    // 1c. CURVED GUIDE RAILS on every belt arc (inner + outer side barriers)
    // These force boxes to follow the curve — realistic side walls
    glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
    for (int b = 0; b < 5; b++) {
        for (size_t si = 0; si < globalPaths[b].size(); si++) {
            const auto& seg = globalPaths[b][si];
            if (seg.type != CURVE) continue;

            float bY      = seg.start.y; // belt surface Y
            float railH   = 0.55f;       // rail height above belt surface
            float railW   = 0.12f;       // rail thickness
            float innerR  = seg.radius - 1.6f; // inner edge radius
            float outerR  = seg.radius + 1.6f; // outer edge radius

            int railSegs = 40;
            for (int s = 0; s < railSegs; s++) {
                float t1 = (float)s       / railSegs;
                float t2 = (float)(s + 1) / railSegs;
                float a1 = seg.startAngle + t1 * seg.sweepAngle;
                float a2 = seg.startAngle + t2 * seg.sweepAngle;

                // Midpoint angle for this segment
                float am = (a1 + a2) * 0.5f;
                float dx = cos(am);
                float dz = sin(am);

                // Chord length for scaling
                glm::vec3 pi1 = seg.center + glm::vec3(cos(a1)*innerR, 0.0f, sin(a1)*innerR);
                glm::vec3 pi2 = seg.center + glm::vec3(cos(a2)*innerR, 0.0f, sin(a2)*innerR);
                float segLen = glm::length(pi2 - pi1);

                // Direction of chord
                glm::vec3 chDir = glm::normalize(pi2 - pi1);
                float chAngle = atan2(chDir.x, chDir.z);

                // Inner rail
                glm::vec3 innerMid = seg.center + glm::vec3(dx * innerR, 0.0f, dz * innerR);
                glm::mat4 innerRail = glm::translate(glm::mat4(1.0f),
                    glm::vec3(innerMid.x, bY + railH * 0.5f, innerMid.z));
                innerRail = glm::rotate(innerRail, chAngle, glm::vec3(0,1,0));
                innerRail = glm::scale(innerRail, glm::vec3(railW, railH, segLen + 0.05f));
                shader.setMat4("model", innerRail);
                glDrawArrays(GL_TRIANGLES, 0, 36);

                // Outer rail
                glm::vec3 outerMid = seg.center + glm::vec3(dx * outerR, 0.0f, dz * outerR);
                glm::mat4 outerRail = glm::translate(glm::mat4(1.0f),
                    glm::vec3(outerMid.x, bY + railH * 0.5f, outerMid.z));
                outerRail = glm::rotate(outerRail, chAngle, glm::vec3(0,1,0));
                outerRail = glm::scale(outerRail, glm::vec3(railW, railH, segLen + 0.05f));
                shader.setMat4("model", outerRail);
                glDrawArrays(GL_TRIANGLES, 0, 36);
            }
        }
    }

    // 1b. SUPPORT PILLARS for elevated belts (belts 1 and 3, Y=4.5)
    glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
    for (int b = 1; b <= 3; b += 2) {         // odd belts only
        float bz     = -40.0f + b * 20.0f;    // -20 or +20
        float beltY  = 4.5f;
        float pillarH = beltY - 0.25f;         // stop BELOW belt underside (Y≈4.25)
        float R = 40.0f;

        // Pillars along the arc: sample 9 evenly-spaced angles
        // sweep = +PI → angles go from PI to 2*PI (curves through -Z)
        for (int p = 0; p <= 8; p++) {
            float frac  = (float)p / 8.0f;
            float angle = glm::pi<float>() + frac * glm::pi<float>(); // PI → 2PI
            float px    = cos(angle) * R;
            float pz    = bz + sin(angle) * R;

            glm::mat4 pillar = glm::translate(glm::mat4(1.0f),
                                              glm::vec3(px, pillarH * 0.5f, pz));
            pillar = glm::scale(pillar, glm::vec3(0.35f, pillarH, 0.35f));
            shader.setMat4("model", pillar);
            glDrawArrays(GL_TRIANGLES, 0, 36);

            // Crossbeam cap at top
            glm::mat4 cap = glm::translate(glm::mat4(1.0f),
                                           glm::vec3(px, pillarH + 0.15f, pz));
            cap = glm::scale(cap, glm::vec3(0.7f, 0.3f, 0.7f));
            shader.setMat4("model", cap);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        // Pillars under straight lead-in (X = -46 to -40) and lead-out (X = 40 to 46)
        for (float sx : {-46.0f, -43.0f, -40.0f, 40.0f, 43.0f, 46.0f}) {
            glm::mat4 pillar = glm::translate(glm::mat4(1.0f),
                                              glm::vec3(sx, pillarH * 0.5f, bz));
            pillar = glm::scale(pillar, glm::vec3(0.35f, pillarH, 0.35f));
            shader.setMat4("model", pillar);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // 2. DRAW MASSIVE BOX POPULATION (Animated)
    extern void getPathPositionAndAngle(int pathIndex, float dist, glm::vec3& outPos, float& outAngle);
    for (size_t i = 0; i < gridBoxes.size(); i++) {
        const GridBox& gb = gridBoxes[i];
        glm::vec3 pos;
        float angle = 0.0f;

        if (gb.state == ON_BELT) {
            getPathPositionAndAngle(gb.pathIndex, gb.distance, pos, angle);
            pos.y += 0.6f;
        } else {
            pos = gb.worldPos;
        }

        if (gb.stage == PAINTED || gb.stage == BOUND) {
            glBindTexture(GL_TEXTURE_2D, blueTex); shader.setVec3("objectColor", glm::vec3(0.2f, 0.4f, 0.8f));
        } else {
            glBindTexture(GL_TEXTURE_2D, boxTex); shader.setVec3("objectColor", glm::vec3(0.8f, 0.6f, 0.4f));
        }

        glm::mat4 model = glm::translate(glm::mat4(1.0f), pos);
        model = glm::rotate(model, angle, glm::vec3(0, 1, 0));
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));
        shader.setMat4("model", model);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        if (gb.stage == BOUND) {
            glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
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

    // 3. DRAW MASSIVE SHELVING
    for (int side = 0; side < 2; side++) {
        for (int t = 0; t < SHELF_TOWERS; t++) {
            float beltZ = -40.0f + (t / 2) * 20.0f;
            float zOff  = (t % 2 == 0) ? -3.0f : 3.0f;
            for (int y = 0; y < SHELF_TIERS; y++) {
                float xPos = (side == 0) ? -52.0f : 52.0f;
                float zPos = beltZ + zOff;
                
                // Shelf Board
                glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
                glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(xPos, y * 3.0f, zPos));
                model = glm::scale(model, glm::vec3(4.0f, 0.2f, 9.0f));
                shader.setMat4("model", model);
                glDrawArrays(GL_TRIANGLES, 0, 36);
                
                // Draw static boxes if occupied
                // Limit to side == 0 (source shelves) so destination shelves don't have overlapping boxes/textures
                if (side == 0) {
                    glBindTexture(GL_TEXTURE_2D, boxTex); shader.setVec3("objectColor", glm::vec3(0.8f, 0.6f, 0.4f));
                    for (int s = 0; s < SHELF_SLOTS; s++) {
                        if (shelfOccupied[side][t][y][s]) {
                            glm::mat4 bModel = glm::translate(glm::mat4(1.0f), shelfSlotPos[side][t][y][s]);
                            bModel = glm::scale(bModel, glm::vec3(1.0f, 1.0f, 1.0f));
                            shader.setMat4("model", bModel);
                            glDrawArrays(GL_TRIANGLES, 0, 36);
                        }
                    }
                }
            }
            
            // Vertical posts
            glBindTexture(GL_TEXTURE_2D, conveyorTex); shader.setVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
            float xPos = (side == 0) ? -52.0f : 52.0f;
            float beltZ2 = -40.0f + (t / 2) * 20.0f;
            float zOff2  = (t % 2 == 0) ? -3.0f : 3.0f;
            float zPos = beltZ2 + zOff2;
            float pxArr[2] = { -1.8f, 1.8f };
            float pzArr[2] = { -4.3f, 4.3f };
            for (int p_i = 0; p_i < 2; p_i++) {
                for (int p_j = 0; p_j < 2; p_j++) {
                    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(xPos + pxArr[p_i], 6.0f, zPos + pzArr[p_j]));
                    model = glm::scale(model, glm::vec3(0.4f, 12.0f, 0.4f));
                    shader.setMat4("model", model);
                    glDrawArrays(GL_TRIANGLES, 0, 36);
                }
            }
        }
    }

    // 3.5 FACILITIES (Applied to all 5 Belts)
    extern float getPathTotalLength(int pathIndex);
    for (int b = 0; b < 5; b++) {
        float len = getPathTotalLength(b);
        // Box Cleaning Stations (2 procedural spinning roller machines per belt)
        // drawCleaningStation(shader, VAO, b, len * 0.25f, conveyorTex, wallTex);
        // drawCleaningStation(shader, VAO, b, len * 0.75f, conveyorTex, wallTex);

        // CENTRAL PAINT CHAMBER
        drawPaintChamber(shader, VAO, b, len * 0.5f, tunnelTex, blueTex, wallTex);
    }

    // 4. DRAW ARMS
    for (int i = 0; i < 20; i++) {
        drawShelfArm(shader, shelfArms[i].basePos, shelfArms[i].effectorPos, conveyorTex, wallTex);
    }

    // 5. DRAW WAREHOUSE ROOM (200x200)
    // 5. DRAW WAREHOUSE ROOM (200x200) - Tiled to fix texture stretching
    glBindTexture(GL_TEXTURE_2D, floorTex); shader.setVec3("objectColor", glm::vec3(0.5f, 0.5f, 0.5f));
    // Draw floor in 20x20 chunks
    for (float x = -90.0f; x <= 90.0f; x += 20.0f) {
        for (float z = -90.0f; z <= 90.0f; z += 20.0f) {
            glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(x, -0.7f, z));
            model = glm::scale(model, glm::vec3(20.0f, 0.1f, 20.0f));
            shader.setMat4("model", model);
            glDrawArrays(GL_TRIANGLES, 0, 36);
            
            // Ceiling
            glm::mat4 cModel = glm::translate(glm::mat4(1.0f), glm::vec3(x, 30.0f, z));
            cModel = glm::scale(cModel, glm::vec3(20.0f, 0.1f, 20.0f));
            shader.setMat4("model", cModel);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // Factory Walls Boundary (Thickness 1, Height 30)
    glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
    
    // Back Wall (Z = -100), Tiled along X
    for (float x = -90.0f; x <= 90.0f; x += 20.0f) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(x, 14.5f, -100.0f));
        model = glm::scale(model, glm::vec3(20.0f, 30.0f, 1.0f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    
    // Front Wall — 25-unit chunks align perfectly with the 50-unit door gap
    // Door gap: X = -25 to +25. Wall covers: -100..-25 (left) and +25..+100 (right).
    // 3 chunks of width 25 each side, centered at ±37.5, ±62.5, ±87.5

    // Left side: centers at -87.5, -62.5, -37.5  (each 25 wide)
    float fwLeftX[] = { -87.5f, -62.5f, -37.5f };
    for (float x : fwLeftX) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(x, 14.5f, 100.0f));
        model = glm::scale(model, glm::vec3(25.0f, 30.0f, 1.0f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    // Right side: centers at 37.5, 62.5, 87.5
    float fwRightX[] = { 37.5f, 62.5f, 87.5f };
    for (float x : fwRightX) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(x, 14.5f, 100.0f));
        model = glm::scale(model, glm::vec3(25.0f, 30.0f, 1.0f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    // Top strip above door (Y: 20..30, height=10) — two 25-unit halves to match tiling
    {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(-12.5f, 25.0f, 100.0f));
        model = glm::scale(model, glm::vec3(25.0f, 10.0f, 1.0f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
        model = glm::translate(glm::mat4(1.0f), glm::vec3(12.5f, 25.0f, 100.0f));
        model = glm::scale(model, glm::vec3(25.0f, 10.0f, 1.0f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }

    // --- 7.5 DYNAMIC B-SPLINE DUCTWORK ---
    extern unsigned int ductVAO;
    extern int ductVertexCount;
    extern unsigned int ductTexture;
    
    glBindVertexArray(ductVAO);
    glBindTexture(GL_TEXTURE_2D, ductTexture);
    
    // High specular for shiny metal exterior
    shader.setFloat("material.shininess", 64.0f);
    shader.setVec3("material.specular", 1.0f, 1.0f, 1.0f);
    
    glm::mat4 ductModel = glm::mat4(1.0f);
    shader.setMat4("model", ductModel);
    glDrawArrays(GL_TRIANGLES, 0, ductVertexCount);
    
    // Restore default specular
    shader.setFloat("material.shininess", 32.0f);
    shader.setVec3("material.specular", 0.3f, 0.3f, 0.3f);

    // --- HELICOID SPIRAL SLIDE --- wrapped around a new pillar in the gap between conveyor belts
    // Re-positioned to X=0.0, Z=20.0, perfectly spaced between Z=0 and Z=40 belts to avoid collisions.
    // Spiral wraps tightly around this pillar from Y=0 (floor) to Y=21 (upper level).
    {
        extern unsigned int helicoidVAO;
        extern int helicoidVertexCount;
        extern unsigned int watchTexture;

        // Helicoid and fractal pillar positioned perfectly in the spacious gap
        glm::vec3 placementPos(0.0f, 0.0f, 20.0f); // Center gap
        glm::vec3 slidePos = placementPos;         // helicoid origin at floor

        // 1. DRAW NEW PILLAR
        glBindVertexArray(VAO);
        glBindTexture(GL_TEXTURE_2D, fracTexture);
        shader.setVec3("objectColor", glm::vec3(0.85f, 0.85f, 0.9f)); // Slightly brighter to pop out
        shader.setFloat("material.shininess", 16.0f);
        
        // Base of the pillar starts strictly at Y=0 so slide wraps correctly 0->21
        glm::mat4 baseM = glm::translate(glm::mat4(1.0f), placementPos);
        // Call with depth=3 to guarantee EXACTLY 2 levels of visible branching (Trunk + 2 splitting levels)
        drawFractalPillar(shader, baseM, 3, 21.0f, 1.5f, 32.0f, 0.0f);

        // 2. DRAW HELICOID SPIRAL SURFACE around that pillar
        glBindVertexArray(helicoidVAO);
        glBindTexture(GL_TEXTURE_2D, watchTexture);
        shader.setVec3("objectColor",   glm::vec3(0.75f, 0.75f, 0.80f));
        shader.setVec3("material.specular",  glm::vec3(1.0f,  1.0f,  1.0f));
        shader.setFloat("material.shininess", 96.0f);
        glm::mat4 slideModel = glm::translate(glm::mat4(1.0f), slidePos);
        shader.setMat4("model", slideModel);
        glDrawArrays(GL_TRIANGLES, 0, helicoidVertexCount);

        // Restore defaults
        shader.setVec3("material.specular",  glm::vec3(0.3f, 0.3f, 0.3f));
        shader.setFloat("material.shininess", 32.0f);
        glBindVertexArray(VAO);
    }

    // --- 7. DYNAMIC B-SPLINE CATWALK ---
    extern unsigned int cwFloorVAO;
    extern int cwFloorVertexCount;
    extern unsigned int cwRailVAO;
    extern int cwRailVertexCount;
    extern std::vector<glm::mat4> catwalkStanchions;
    extern unsigned int cylinderVAO;
    extern int cylinderVertexCount;

    // Draw Floor
    glBindVertexArray(cwFloorVAO);
    glBindTexture(GL_TEXTURE_2D, walkTexture);
    shader.setMat4("model", glm::mat4(1.0f));
    glDrawArrays(GL_TRIANGLES, 0, cwFloorVertexCount);

    // Draw Rails
    glBindVertexArray(cwRailVAO);
    glBindTexture(GL_TEXTURE_2D, blueTex); shader.setVec3("objectColor", glm::vec3(0.2f, 0.4f, 0.8f)); 
    shader.setMat4("model", glm::mat4(1.0f));
    glDrawArrays(GL_TRIANGLES, 0, cwRailVertexCount);

    // Draw Support Stanchions
    glBindVertexArray(cylinderVAO);
    glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
    for (const glm::mat4& sM : catwalkStanchions) {
        shader.setMat4("model", sM);
        glDrawArrays(GL_TRIANGLES, 0, cylinderVertexCount);
    }
    
    // Support columns hitting floor
    glBindVertexArray(VAO);
    glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
    // Columns under the high flat sections
    for (float px : {-60.0f, 0.0f, 40.0f}) {
        glm::mat4 cM = glm::translate(glm::mat4(1.0f), glm::vec3(px, 8.25f, -15.0f));
        cM = glm::scale(cM, glm::vec3(1.5f, 16.5f, 1.5f));
        shader.setMat4("model", cM); glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    
    // The Sliding Door  (width=50, height=20, gap from X=-25 to +25)
    extern float doorYOffset;
    glBindTexture(GL_TEXTURE_2D, tunnelTex);
    glm::mat4 doorModel = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 10.0f + doorYOffset, 100.0f));
    doorModel = glm::scale(doorModel, glm::vec3(50.0f, 20.0f, 1.1f));
    shader.setMat4("model", doorModel); glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
    // Right Wall, Tiled along Z
    for (float z = -90.0f; z <= 90.0f; z += 20.0f) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(100.0f, 14.5f, z));
        model = glm::scale(model, glm::vec3(1.0f, 30.0f, 20.0f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    
    // Left Wall, Tiled along Z with structured cutout
    for (float z = -90.0f; z <= 90.0f; z += 20.0f) {
        if (abs(z - 10.0f) < 0.1f || abs(z + 10.0f) < 0.1f) {
            continue; // Skip the z=-10 and z=10 segments (total width 40 around center Z=0)
        }
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(-100.0f, 14.5f, z));
        model = glm::scale(model, glm::vec3(1.0f, 30.0f, 20.0f));
        shader.setMat4("model", model); glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    
    // Custom Left Wall Frame for Exhaust Fan cutout
    // Fan Center is lowered to Y=19. Frame Hole is now Y in [11, 27], Z in [-8, 8]. (Width=16, Height=16)
    // Gap filled: Z in [-20, 20]. Width=40.

    // Bottom Block: covers Y from -0.5 to 11. Height = 11.5. Center Y = 5.25
    glm::mat4 bModel = glm::translate(glm::mat4(1.0f), glm::vec3(-100.0f, 5.25f, 0.0f));
    bModel = glm::scale(bModel, glm::vec3(1.0f, 11.5f, 40.0f));
    shader.setMat4("model", bModel); glDrawArrays(GL_TRIANGLES, 0, 36);
    
    // Top Block: covers Y from 27 to 29.5. Height = 2.5. Center Y = 28.25. (Ducts at Y=28 pierce flawlessly through this thinner wall seal)
    glm::mat4 tModel = glm::translate(glm::mat4(1.0f), glm::vec3(-100.0f, 28.25f, 0.0f));
    tModel = glm::scale(tModel, glm::vec3(1.0f, 2.5f, 40.0f));
    shader.setMat4("model", tModel); glDrawArrays(GL_TRIANGLES, 0, 36);

    // Left Block: covers Z from -20 to -8. Width = 12. Center Z = -14. Y=19, Height=16.
    glm::mat4 lModel = glm::translate(glm::mat4(1.0f), glm::vec3(-100.0f, 19.0f, -14.0f));
    lModel = glm::scale(lModel, glm::vec3(1.0f, 16.0f, 12.0f));
    shader.setMat4("model", lModel); glDrawArrays(GL_TRIANGLES, 0, 36);

    // Right Block: covers Z from 8 to 20. Width = 12. Center Z = 14. Y=19, Height=16.
    glm::mat4 rModel = glm::translate(glm::mat4(1.0f), glm::vec3(-100.0f, 19.0f, 14.0f));
    rModel = glm::scale(rModel, glm::vec3(1.0f, 16.0f, 12.0f));
    shader.setMat4("model", rModel); glDrawArrays(GL_TRIANGLES, 0, 36);
    
    // Render the Exhaust Fan inside the custom cut
    drawExhaustFan(shader);

    // --- 6. RANDOM MATERIALS STORAGE ---
    //// Storing miscellaneous materials near the Back Wall (Z = -95)
    //for (float x = -80.0f; x <= 80.0f; x += 15.0f) {
    //    float heightSeed = abs(sin(x * 12.3f) + cos(x * 4.5f)); // pseudo-random
    //    int numStacks = 1 + (int)(heightSeed * 3.0f); // 1 to 4 boxes stacked

    //    if (numStacks % 2 == 0) {
    //        // Draw stacked crates
    //        glBindTexture(GL_TEXTURE_2D, boxTex); shader.setVec3("objectColor", glm::vec3(0.8f, 0.6f, 0.4f));
    //        for (int y = 0; y < numStacks; y++) {
    //            glm::mat4 crate = glm::translate(glm::mat4(1.0f), glm::vec3(x, y * 2.0f + 0.3f, -95.0f));
    //            crate = glm::rotate(crate, heightSeed, glm::vec3(0,1,0)); // random rotation
    //            shader.setMat4("model", glm::scale(crate, glm::vec3(2.0f, 2.0f, 2.0f)));
    //            glDrawArrays(GL_TRIANGLES, 0, 36);
    //        }
    //    } else {
    //        // Draw proper barrel shapes: bulging body + caps + metal band rings
    //        extern unsigned int barrelVAO;
    //        extern int barrelVertexCount;
    //        extern unsigned int barrelTexture;

    //        const float bScale = 2.0f;   // diameter
    //        const float bH     = 2.3f;   // height of each barrel unit

    //        for (int iy = 0; iy < 2; iy++) {         // 2 high
    //            for (int iz = 0; iz < 2; iz++) {     // 2 deep
    //                float bz = -93.0f - iz * (bScale + 0.3f);
    //                float by = iy * (bH + 0.05f) + bH * 0.5f;  // bottom sits on floor

    //                // ── barrel body ─────────────────────────────────────────
    //                glBindVertexArray(barrelVAO);
    //                glBindTexture(GL_TEXTURE_2D, barrelTexture);
    //                shader.setVec3("objectColor", glm::vec3(0.55f, 0.35f, 0.20f));
    //                shader.setFloat("material.shininess", 16.0f);
    //                glm::mat4 bm = glm::translate(glm::mat4(1.0f), glm::vec3(x, by, bz));
    //                bm = glm::scale(bm, glm::vec3(bScale, bH, bScale));
    //                shader.setMat4("model", bm);
    //                glDrawArrays(GL_TRIANGLES, 0, barrelVertexCount);

    //                // ── 3 metal band rings per barrel ────────────────────────
    //                glBindVertexArray(VAO);
    //                glBindTexture(GL_TEXTURE_2D, conveyorTex);
    //                shader.setVec3("objectColor", glm::vec3(0.32f, 0.36f, 0.38f));
    //                float bandOffsets[3] = { -0.34f, 0.0f, 0.34f };
    //                for (int bi = 0; bi < 3; bi++) {
    //                    glm::mat4 band = glm::translate(glm::mat4(1.0f),
    //                        glm::vec3(x, by + bandOffsets[bi] * bH, bz));
    //                    band = glm::scale(band, glm::vec3(bScale * 1.07f, 0.09f, bScale * 1.07f));
    //                    shader.setMat4("model", band);
    //                    glDrawArrays(GL_TRIANGLES, 0, 36);
    //                }
    //            }
    //        }
    //        shader.setFloat("material.shininess", 32.0f);
    //        glBindVertexArray(VAO);
    //    }
    //}

    //// ★ DECORATIVE OVALS ON FLOOR ★ at specified WCS locations
    //extern unsigned int ovalVAO;
    //extern int ovalVertexCount;
    //glBindVertexArray(ovalVAO);
    //glBindTexture(GL_TEXTURE_2D, bakaTexture);
    //
    //// Shape created from provided WCS coordinates - placed near back wall at Z=5.1
    //// Coordinates span from X: -0.8 to -0.7 (approximately), Y: 0.408 to 2.149
    //// This creates an oval/shape near the back wall
    //glm::mat4 bakaShape = glm::translate(glm::mat4(1.0f), glm::vec3(-0.8f, 0.0f, 5.1f));
    //bakaShape = glm::scale(bakaShape, glm::vec3(4.0f, 1.0f, 3.0f));
    //shader.setMat4("model", bakaShape);
    //glDrawArrays(GL_TRIANGLE_FAN, 0, ovalVertexCount);
    //
    //// ★ CONE SHAPE ON FLOOR ★ next to baka shape
    //extern unsigned int coneVAO;
    //extern int coneVertexCount;
    //glBindVertexArray(coneVAO);
    //glBindTexture(GL_TEXTURE_2D, coneTexture);
    //
    //// Cone positioned beside the baka shape
    //glm::mat4 coneShape = glm::translate(glm::mat4(1.0f), glm::vec3(1.2f, 0.0f, 5.1f));
    //coneShape = glm::scale(coneShape, glm::vec3(2.5f, 1.0f, 2.5f));
    //shader.setMat4("model", coneShape);
    //glDrawArrays(GL_TRIANGLE_STRIP, 0, coneVertexCount);
    
    // 7. DRAW PARTICLES
    extern std::vector<Particle> sparkParticles;
    if (!sparkParticles.empty()) {
        glBindVertexArray(VAO); 
        glBindTexture(GL_TEXTURE_2D, whiteLightTex); // glowing color
        shader.setBool("ambientOn", false); 
        shader.setBool("useTextureColorOnly", true);
        
        for (const auto& p : sparkParticles) {
            glm::mat4 pM = glm::translate(glm::mat4(1.0f), p.position);
            float scale = 0.15f * (p.life / p.initialLife);
            pM = glm::scale(pM, glm::vec3(scale));
            shader.setMat4("model", pM);
            glDrawArrays(GL_TRIANGLES, 0, 36); // basic cube
        }
        shader.setBool("ambientOn", ambientOn); 
        shader.setBool("useTextureColorOnly", useTextureColorOnly);
    shader.setBool("textureOn", textureOn); 
    }



    // 10. MENGER SPONGE (Automated Modular Storage Racks)
    {
        extern unsigned int mengerVAO;
        extern int mengerVertexCount;
        
        glBindVertexArray(mengerVAO);
        glBindTexture(GL_TEXTURE_2D, wallTex); shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f));
        
        shader.setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.65f)); // iron gray
        shader.setFloat("material.shininess", 16.0f);
        
        // Massive shelving unit near the back right corner: X=70, Z=-80. Floor is Y=0.
        // Base is -0.5 to 0.5. Height 30 means it spans Y=-15 to Y=15. Move it up by half height!
        glm::mat4 spongeModel = glm::translate(glm::mat4(1.0f), glm::vec3(70.0f, 15.0f, -80.0f));
        spongeModel = glm::scale(spongeModel, glm::vec3(30.0f, 30.0f, 15.0f)); 
        
        shader.setMat4("model", spongeModel);
        glDrawArrays(GL_TRIANGLES, 0, mengerVertexCount);

    }

    // ═══════════════════════════════════════════════════════════════
    //   OUTDOOR ENVIRONMENT  —  street, grass (all 4 sides), sky
    // ═══════════════════════════════════════════════════════════════
    extern unsigned int streetTexture, grassTexture, skyTexture;
    extern bool useTextureColorOnly;

    glBindVertexArray(VAO);
    glActiveTexture(GL_TEXTURE0);

    // Ground tiles use identical Y / scale as the indoor floor so they
    // sit flush at the building walls with zero gaps.
    const float GY = -0.7f;   // same centre-Y as indoor floor
    const float GH =  0.1f;   // same height scale

    // ── 1. STREET ──────────────────────────────────────────────────
    // 50-unit wide corridor (X: -25 → +25) aligned with the front
    // door gap, extending from Z=100 outward to Z=400.
    // Two 25-wide tiles per row tile the corridor exactly.
    glBindTexture(GL_TEXTURE_2D, streetTexture);
    shader.setVec3("objectColor", glm::vec3(0.35f, 0.35f, 0.35f));
    for (float z = 110.0f; z <= 390.0f; z += 20.0f) {
        for (float x : {-12.5f, 12.5f}) {   // covers X –25..+25
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, GY, z));
            m = glm::scale(m, glm::vec3(25.0f, GH, 20.0f));
            shader.setMat4("model", m);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
    }

    // ── 2. GRASS — wraps all 4 sides of the 200×200 building ───────
    glBindTexture(GL_TEXTURE_2D, grassTexture);
    shader.setVec3("objectColor", glm::vec3(0.25f, 0.60f, 0.25f));

    // FRONT-LEFT  (X: -25 → -400,  Z: 100 → 400)
    // Tile at X=-35 has its right edge at X=-25, butting up against the street.
    for (float z = 110.0f; z <= 390.0f; z += 20.0f)
        for (float x = -35.0f; x >= -395.0f; x -= 20.0f) {
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, GY, z));
            m = glm::scale(m, glm::vec3(20.0f, GH, 20.0f));
            shader.setMat4("model", m);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

    // FRONT-RIGHT  (X: 25 → 400,  Z: 100 → 400)
    // Tile at X=35 has its left edge at X=25.
    for (float z = 110.0f; z <= 390.0f; z += 20.0f)
        for (float x = 35.0f; x <= 395.0f; x += 20.0f) {
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, GY, z));
            m = glm::scale(m, glm::vec3(20.0f, GH, 20.0f));
            shader.setMat4("model", m);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

    // BACK ZONE  (full width X: -400 → 400,  Z: -100 → -400)
    // Tile at Z=-110 has its upper  edge at Z=-100 (back wall).
    for (float z = -110.0f; z >= -390.0f; z -= 20.0f)
        for (float x = -390.0f; x <= 390.0f; x += 20.0f) {
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, GY, z));
            m = glm::scale(m, glm::vec3(20.0f, GH, 20.0f));
            shader.setMat4("model", m);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

    // LEFT SIDE  (X: -100 → -400,  Z: -100 → 100)
    // Tile at X=-110 has its right edge at X=-100 (left wall).
    // Z range matches indoor floor tiles so they meet seamlessly.
    for (float x = -110.0f; x >= -390.0f; x -= 20.0f)
        for (float z = -90.0f; z <= 90.0f; z += 20.0f) {
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, GY, z));
            m = glm::scale(m, glm::vec3(20.0f, GH, 20.0f));
            shader.setMat4("model", m);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

    // RIGHT SIDE  (X: 100 → 400,  Z: -100 → 100)
    // Tile at X=110 has its left edge at X=100 (right wall).
    for (float x = 110.0f; x <= 390.0f; x += 20.0f)
        for (float z = -90.0f; z <= 90.0f; z += 20.0f) {
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, GY, z));
            m = glm::scale(m, glm::vec3(20.0f, GH, 20.0f));
            shader.setMat4("model", m);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

    // ── LIGHT FIXTURES ─────────────────────────────────────────────
    // Call the built-in geometry renderer for the ceiling fixtures 
    // to stick them to the roof and drop realistic cords down to point light coordinates
    drawCeilingLights(shader, VAO, wallTex, whiteLightTex);

    // ── 3. SKY ─────────────────────────────────────────────────────
    // Inverted sphere (Skydome) tracked to the camera position so it is always
    // centred on the viewer and never clipped by the far plane.
    // Scale 1800 → radius 900 from camera, well within zFar=1000.
    // Drawn last with depth-write OFF so it fills background pixels only.
    extern unsigned int sphereVAO;
    extern int sphereVertexCount;
    
    glDepthMask(GL_FALSE);
    glFrontFace(GL_CW);   // invert winding so we see the inside faces
    glBindVertexArray(sphereVAO);
    glBindTexture(GL_TEXTURE_2D, skyTexture);
    shader.setVec3("objectColor", glm::vec3(0.53f, 0.81f, 0.98f));
    shader.setBool("useTextureColorOnly", true);
    
    shader.setBool("isSkybox", true);
    
    glm::mat4 skyModel = glm::translate(glm::mat4(1.0f), camPos);  // follow camera
    skyModel = glm::scale(skyModel, glm::vec3(1800.0f, 1800.0f, 1800.0f));
    shader.setMat4("model", skyModel);
    glDrawArrays(GL_TRIANGLES, 0, sphereVertexCount);
    
    shader.setBool("isSkybox", false);
    
    shader.setBool("useTextureColorOnly", useTextureColorOnly);
    glFrontFace(GL_CCW);
    glDepthMask(GL_TRUE);

    glBindVertexArray(VAO);
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
        
        // E for UP, X for DOWN
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) mainCamera.ProcessKeyboard(UP, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) mainCamera.ProcessKeyboard(DOWN, deltaTime);
    }

    // Assignment mode specific controls
    if (mainCamera.Mode == ASSIGNMENT) {
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) mainCamera.ProcessKeyboard(DOWN, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS) mainCamera.ProcessKeyboard(YAW_LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) mainCamera.ProcessKeyboard(ROLL_LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) mainCamera.ProcessKeyboard(ROTATE_AROUND, deltaTime);
    }

    // Exhaust Fan Speed Control (Accelerate and Decelerate smartly)
    extern float exhaustFanSpeed;
    if (glfwGetKey(window, GLFW_KEY_KP_MULTIPLY) == GLFW_PRESS || 
       (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS && (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS))) {
        exhaustFanSpeed += 20.0f * deltaTime;
        if (exhaustFanSpeed > 50.0f) exhaustFanSpeed = 50.0f; // Max velocity cap
    }
    if (glfwGetKey(window, GLFW_KEY_KP_DIVIDE) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_SLASH) == GLFW_PRESS) {
        exhaustFanSpeed -= 20.0f * deltaTime;
        if (exhaustFanSpeed < -50.0f) exhaustFanSpeed = -50.0f; // Max reverse velocity cap
    }

    // Conveyor Belt Box Speed Control  (+ to speed up, - to slow down)
    extern float conveyorSpeed;
    if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS) {
        conveyorSpeed += 5.0f * deltaTime;
        if (conveyorSpeed > 20.0f) conveyorSpeed = 20.0f;  // cap: boxes don't overshoot collision checks
    }
    if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS) {
        conveyorSpeed -= 5.0f * deltaTime;
        if (conveyorSpeed < 0.5f) conveyorSpeed = 0.5f;    // floor: keep boxes crawling, never dead-stopped
    }

    // 90-degree Snap Camera Rotation
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        if (!keyLeft_pressed) {
            mainCamera.RotateYaw(-45.0f);
            keyLeft_pressed = true;
        }
    } else { keyLeft_pressed = false; }

    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        if (!keyRight_pressed) {
            mainCamera.RotateYaw(45.0f);
            keyRight_pressed = true;
        }
    } else { keyRight_pressed = false; }

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
    // L key: Master light toggle — also resets P override so darkness is pure
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) {
        if (!keyL_pressed) {
            masterLightOn = !masterLightOn;
            p_override_light = false; // Reset so scene goes fully dark
            keyL_pressed = true;
        }
    } else { keyL_pressed = false; }
    DO_TOGGLE(GLFW_KEY_M, mainLightOn, keyM_pressed)
    DO_TOGGLE(GLFW_KEY_G, fanOn, keyG_pressed)
    DO_TOGGLE(GLFW_KEY_V, singleViewport, keyV_pressed)
    // T key: Texture Only Mode (Texture ON, Surface Lighting OFF)
    if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
        if (!keyT_pressed) {
            useTextureColorOnly = !useTextureColorOnly;
            if (useTextureColorOnly) textureOn = true; // Ensure texture is visible
            keyT_pressed = true;
        }
    } else { keyT_pressed = false; }

    // R key: Vanilla Shading Mode (Texture OFF, Surface Lighting ON)
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
        if (!keyR_pressed) {
            textureOn = !textureOn;
            if (!textureOn) useTextureColorOnly = false; // Ensure we see surface shading
            keyR_pressed = true;
        }
    } else { keyR_pressed = false; }

    // P key: Toggle Gouraud/Phong. Also enables ambient light override so
    //         vertex/fragment shading is visible even when master light is OFF.
    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
        if (!keyP_pressed) {
            usePhong = !usePhong;
            useTextureColorOnly = false; // Show blended result, not texture-only
            textureOn = true;            // Ensure texture is part of the blend
            if (!masterLightOn) p_override_light = true; // Allow dim ambient when dark
            keyP_pressed = true;
        }
    } else { keyP_pressed = false; }
    DO_TOGGLE(GLFW_KEY_O, doorOpen, keyO_pressed)

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

