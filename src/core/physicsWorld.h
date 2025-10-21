#pragma once
#include "rigidbody.h"
#include "../collision/collision.h"
#include <vector>
#include <GLFW/glfw3.h>
#include <cmath>

class PhysicsWorld {
public:
    std::vector<RigidBody> bodies;

    void AddBody(const RigidBody& body) {
        bodies.push_back(body);
    }

    void Update(float dt, float gravity, float viscosity, float boundCor) {
        for (auto& body : bodies) {
            UpdatePhysics(body, dt, gravity, viscosity, boundCor);
        }

        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                ContactPoint contact;
                if (CheckCollision(bodies[i], bodies[j], &contact)) {
                    ResolveCollision(bodies[i], bodies[j], contact, 1.0f);
                }
            }
        }
    }

    void Render() {
        for (const auto& body : bodies) {
            RenderBody(body);
        }
    }

float CalculateTotalEnergy(float gravity) {
    float totalEnergy = 0.0f;
    for (const auto& body : bodies) {
        float vSquared = body.vx * body.vx + body.vy * body.vy;
        float kineticEnergy = 0.5f * body.mass * vSquared;
        
        float rotationalEnergy = 0.5f * body.momentOfInertia * body.angularVelocity * body.angularVelocity;
        
        float height = 48.0f - body.y;
        float potentialEnergy = body.mass * gravity * height;
        
        totalEnergy += kineticEnergy + rotationalEnergy + potentialEnergy;
    }
    return totalEnergy;
}

private:
    void RenderBody(const RigidBody& body) {
        glPushMatrix();
        glTranslatef(body.x, body.y, 0.0f);
        glRotatef(body.angle * 180.0f / 3.14159265f, 0.0f, 0.0f, 1.0f);
        
        if (body.shape == ShapeType::RECTANGLE) {
            float halfWidth = body.width / 2.0f;
            float halfHeight = body.height / 2.0f;
            glBegin(GL_QUADS);
            glVertex2f(-halfWidth, -halfHeight);
            glVertex2f(halfWidth, -halfHeight);
            glVertex2f(halfWidth, halfHeight);
            glVertex2f(-halfWidth, halfHeight);
            glEnd();
        }
        else if (body.shape == ShapeType::ELLIPSE) {
            float xRadius = body.width / 2.0f;
            float yRadius = body.height / 2.0f;
            int segments = body.detail;
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(0.0f, 0.0f);
            for (int i = 0; i <= segments; i++) {
                float theta = 2.0f * 3.1415926f * float(i) / float(segments);
                float dx = xRadius * cosf(theta);
                float dy = yRadius * sinf(theta);
                glVertex2f(dx, dy);
            }
            glEnd();
        }
        
        glPopMatrix();
    }

    void UpdatePhysics(RigidBody& body, float dt, float gravity, float viscosity, float boundCor) {
        body.vx *= (1.0f - viscosity);
        body.vy *= (1.0f - viscosity);
        body.angularVelocity *= (1.0f - viscosity);
        
        body.vy += gravity * dt;
        
        body.x += body.vx * dt;
        body.y += body.vy * dt;
        body.angle += body.angularVelocity * dt;

        if (body.y + body.height / 2.0f >= SCREEN_HEIGHT) {
            body.y = SCREEN_HEIGHT - body.height / 2.0f;
            body.vy = -body.vy * boundCor;
        }
        if (body.x - body.width / 2.0f <= 0.0f) {
            body.x = body.width / 2.0f;
            body.vx = -body.vx * boundCor;
        }
        if (body.x + body.width / 2.0f >= SCREEN_WIDTH) {
            body.x = SCREEN_WIDTH - body.width / 2.0f;
            body.vx = -body.vx * boundCor;
        }
        if (body.y - body.height / 2.0f <= 0.0f) {
            body.y = body.height / 2.0f;
            body.vy = -body.vy * boundCor;
        }
    }

};