#pragma once
#include "rigidbody.h"
#include "../collision/collision.h"
#include <vector>
#include <GLFW/glfw3.h>

class PhysicsWorld {
public:
    std::vector<RigidBody> bodies;

    void AddBody(const RigidBody& body) {
        bodies.push_back(body);
    }

    void Update(float dt) {
        for (auto& body : bodies) {
            UpdatePhysics(body, dt, 0.000018, 1.0f);
        }

        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                if (CheckCollision(bodies[i], bodies[j])) {
                    ResolveCollision(bodies[i], bodies[j]);
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
        if (body.shape == ShapeType::RECTANGLE) {
            float halfWidth = body.width / 2.0f;
            float halfHeight = body.height / 2.0f;
            glBegin(GL_QUADS);
            glVertex2f(body.x - halfWidth, body.y - halfHeight);
            glVertex2f(body.x + halfWidth, body.y - halfHeight);
            glVertex2f(body.x + halfWidth, body.y + halfHeight);
            glVertex2f(body.x - halfWidth, body.y + halfHeight);
            glEnd();
        }
        else if (body.shape == ShapeType::ELLIPSE32) {
            float xRadius = body.width / 2.0f;
            float yRadius = body.height / 2.0f;
            int segments = 32;
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(body.x, body.y);
            for (int i = 0; i <= segments; i++) {
                float theta = 2.0f * 3.1415926f * float(i) / float(segments);
                float dx = xRadius * cosf(theta);
                float dy = yRadius * sinf(theta);
                glVertex2f(body.x + dx, body.y + dy);
            }
            glEnd();
        }
    }

    void UpdatePhysics(RigidBody& body, float dt, float viscosity, float boundCOR) {
        body.vy += 9.8f * dt;
        body.vx -= viscosity * body.vx * body.vx * dt;
        body.vy -= viscosity * body.vy * body.vy * dt;
        body.x += body.vx * dt;
        body.y += body.vy * dt;

        if (body.y + body.height / 2 >= 48) {
            body.vy *= -boundCOR;
            body.y = 48 - body.height / 2;
        }
        if (body.y - body.height / 2 <= 0) {
            body.vy *= -boundCOR;
            body.y = body.height / 2;
        }
        if (body.x + body.width / 2 >= 64) {
            body.vx *= -boundCOR;
            body.x = 64 - body.width / 2;
        }
        if (body.x - body.width / 2 <= 0) {
            body.vx *= -boundCOR;
            body.x = body.width / 2;
        }
    }


};
