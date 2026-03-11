#include "physicsWorld.h"
#include "joints/distanceJoint.h"
#include <cmath>

PhysicsWorld::PhysicsWorld() = default;
PhysicsWorld::~PhysicsWorld() = default;

int PhysicsWorld::AddBody(const RigidBody& body) {
    bodies.push_back(body);
    bodies[bodies.size()-1].id = bodies.size()-1;
    return bodies.size()-1;
}

void PhysicsWorld::AddDistanceJoint(const DistanceJoint& joint) {
    distanceJoints.push_back(joint);
}

void PhysicsWorld::Update(float dt, float gravity, float viscosity, float boundCor) {
    for (auto& body : bodies) {
        UpdatePhysics(body, dt, gravity, viscosity, boundCor);
    }
    for (auto& joint: distanceJoints){
        UpdateDistanceJoint(joint, dt);
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

void PhysicsWorld::Render() {
    for (const auto& body : bodies) {
        RenderBody(body);
    }
}

void PhysicsWorld::SetAnchor(int id, bool anchor){
    bodies[id].anchor = anchor;
}

float PhysicsWorld::CalculateTotalEnergy(float gravity) {
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

void PhysicsWorld::RenderBody(const RigidBody& body) {
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

void PhysicsWorld::UpdatePhysics(RigidBody& body, float dt, float gravity, float viscosity, float boundCor) {
    body.vx *= (1.0f - viscosity);
    body.vy *= (1.0f - viscosity);
    body.angularVelocity *= (1.0f - viscosity);
    
    body.vy += gravity * dt * body.gravityScale;
    
    if (!body.anchor)
    {
        body.x += body.vx * dt;
        body.y += body.vy * dt;
    }
    
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

void PhysicsWorld::UpdateDistanceJoint(DistanceJoint& joint, float dt){
    RigidBody& bodyA = bodies[joint.bodyAId];
    RigidBody& bodyB = bodies[joint.bodyBId];
    float dx = bodyA.x - bodyB.x;
    float dy = bodyA.y - bodyB.y;
    float distance = pow((dx*dx)+(dy*dy), 0.5f);
    float x = distance-joint.length;
    if (abs(x) < 0.0001f) return;
    float force = -joint.strength*x;
    float bodyBaccl = -force/bodyB.mass;
    float bodyBacclX = bodyBaccl*(dx/distance);
    float bodyBacclY = bodyBaccl*(dy/distance);
    bodyB.vx += bodyBacclX*dt;
    bodyB.vy += bodyBacclY*dt;
    if (!joint.anchorA)
    {
        float bodyAaccl = force/bodyA.mass;
        float bodyAacclX = bodyAaccl*(dx/distance);
        float bodyAacclY = bodyAaccl*(dy/distance);
        bodyA.vx += bodyAacclX*dt;
        bodyA.vy += bodyAacclY*dt;
    }
}
