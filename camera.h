#ifndef CAMERA_H
#define CAMERA_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

enum CameraMode {
    ASSIGNMENT,
    REALISTIC,
    BIRD_EYE,
    FOLLOW
};

enum CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN,
    PITCH_UP,
    PITCH_DOWN,
    YAW_LEFT,
    YAW_RIGHT,
    ROLL_LEFT,
    ROLL_RIGHT,
    ROTATE_AROUND
};

const float YAW = -90.0f;
const float PITCH = 0.0f;
const float ROLL = 0.0f;
const float SPEED = 30.0f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera {
public:
    CameraMode Mode;

    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    glm::vec3 LookAtPoint;

    float Yaw;
    float Pitch;
    float Roll;

    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    glm::vec3 FollowTarget;
    glm::vec3 FollowOffset;

    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH)
        : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM), Mode(REALISTIC)
    {
        Position = position;
        WorldUp = up;
        Yaw = yaw;
        Pitch = pitch;
        Roll = ROLL;
        LookAtPoint = glm::vec3(0.0f, 0.0f, 0.0f);
        FollowTarget = glm::vec3(0.0f, 0.0f, 0.0f);
        FollowOffset = glm::vec3(0.0f, 5.0f, 10.0f);
        updateCameraVectors();
    }

    glm::mat4 GetViewMatrix() {
        if (Mode == BIRD_EYE) {
            return glm::lookAt(glm::vec3(0.0f, 20.0f, 0.1f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        }
        else if (Mode == FOLLOW) {
            glm::vec3 targetPos = FollowTarget;
            glm::vec3 camPos = targetPos + FollowOffset;
            return glm::lookAt(camPos, targetPos, glm::vec3(0.0f, 1.0f, 0.0f));
        }
        else {
            // Apply roll
            glm::vec3 rollUp = glm::rotate(Up, glm::radians(Roll), Front);
            return glm::lookAt(Position, Position + Front, rollUp);
        }
    }

    void ProcessKeyboard(CameraMovement direction, float deltaTime) {
        float velocity = MovementSpeed * deltaTime;
        float rotationVelocity = 45.0f * deltaTime; // 45 degrees per second

        if (Mode == ASSIGNMENT || Mode == REALISTIC) {
            if (direction == FORWARD) Position += Front * velocity;
            if (direction == BACKWARD) Position -= Front * velocity;
            if (direction == LEFT) Position -= Right * velocity;
            if (direction == RIGHT) Position += Right * velocity;
        }

        if (Mode == ASSIGNMENT) {
            if (direction == UP) Position += WorldUp * velocity;
            if (direction == DOWN) Position -= WorldUp * velocity;
            if (direction == PITCH_UP) Pitch += rotationVelocity;
            if (direction == PITCH_DOWN) Pitch -= rotationVelocity;
            if (direction == YAW_LEFT) Yaw -= rotationVelocity;
            if (direction == YAW_RIGHT) Yaw += rotationVelocity;
            if (direction == ROLL_LEFT) Roll -= rotationVelocity;
            if (direction == ROLL_RIGHT) Roll += rotationVelocity;
            if (direction == ROTATE_AROUND) {
                // simple pivot around LookAtPoint
                glm::vec3 dir = Position - LookAtPoint;
                dir = glm::rotate(dir, glm::radians(rotationVelocity), WorldUp);
                Position = LookAtPoint + dir;
                // keep looking at the point
                glm::vec3 lookF = glm::normalize(LookAtPoint - Position);
                Pitch = glm::degrees(asin(lookF.y));
                Yaw = glm::degrees(atan2(lookF.z, lookF.x));
            }
        }

        if (Mode == REALISTIC) {
            if (direction == ROLL_LEFT) Roll -= rotationVelocity;
            if (direction == ROLL_RIGHT) Roll += rotationVelocity;
        }

        // constrain pitch
        if (Pitch > 89.0f) Pitch = 89.0f;
        if (Pitch < -89.0f) Pitch = -89.0f;

        updateCameraVectors();
    }

    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true) {
        if (Mode == REALISTIC) {
            xoffset *= MouseSensitivity;
            yoffset *= MouseSensitivity;

            Yaw += xoffset;
            Pitch += yoffset;

            if (constrainPitch) {
                if (Pitch > 89.0f) Pitch = 89.0f;
                if (Pitch < -89.0f) Pitch = -89.0f;
            }

            updateCameraVectors();
        }
    }

    void SetTarget(glm::vec3 target) {
        FollowTarget = target;
    }

private:
    void updateCameraVectors() {
        glm::vec3 front;
        front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front.y = sin(glm::radians(Pitch));
        front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        Front = glm::normalize(front);
        Right = glm::normalize(glm::cross(Front, WorldUp));
        Up = glm::normalize(glm::cross(Right, Front));
    }
};

#endif
