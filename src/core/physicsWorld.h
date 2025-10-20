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
                    ResolveCollision(bodies[i], bodies[j], contact, 0.8f);
                }
            }
        }
    }

    void Render() {
        for (const auto& body : bodies) {
            RenderBody(body);
        }
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

    void UpdatePhysics(RigidBody& body, float dt, float gravity, float viscosity, float boundCOR) {
        body.vy += gravity * dt;
        
        float dragFactor = 1.0f - viscosity * dt;
        if (dragFactor < 0.0f) dragFactor = 0.0f;
        body.vx *= dragFactor;
        body.vy *= dragFactor;
        
        body.angularVelocity *= dragFactor;
        
        body.x += body.vx * dt;
        body.y += body.vy * dt;
        body.angle += body.angularVelocity * dt;
        
        float halfWidth = body.width / 2.0f;
        float halfHeight = body.height / 2.0f;
        
        float boundingRadius = sqrtf(halfWidth * halfWidth + halfHeight * halfHeight);
        
        if (body.y + boundingRadius >= 48) {
            body.vy *= -boundCOR;
            body.angularVelocity *= boundCOR;
            body.y = 48 - boundingRadius;
        }
        if (body.y - boundingRadius <= 0) {
            body.vy *= -boundCOR;
            body.angularVelocity *= boundCOR;
            body.y = boundingRadius;
        }
        if (body.x + boundingRadius >= 64) {
            body.vx *= -boundCOR;
            body.angularVelocity *= boundCOR;
            body.x = 64 - boundingRadius;
        }
        if (body.x - boundingRadius <= 0) {
            body.vx *= -boundCOR;
            body.angularVelocity *= boundCOR;
            body.x = boundingRadius;
        }
    }
};