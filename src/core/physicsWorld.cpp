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

void PhysicsWorld::Update(float dt, float gravity, float dragCoeff, float boundCor) {
    for (auto& joint: distanceJoints){
        UpdateDistanceJoint(joint, dt);
    }
    for (auto& body : bodies) {
        UpdatePhysics(body, dt, gravity, dragCoeff, boundCor);
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
    for (auto& body : bodies) {
        totalEnergy += body.calculateEnergy(gravity);
    }
    for (auto& joint : distanceJoints){
        totalEnergy += joint.calculateEnergy(bodies[joint.bodyAId].pos, bodies[joint.bodyBId].pos);
    }
    return totalEnergy;
}

void PhysicsWorld::RenderBody(const RigidBody& body) {
    glPushMatrix();
    glTranslatef(body.pos.x, body.pos.y, 0.0f);
    glRotatef(body.angle * 180.0f / 3.14159265f, 0.0f, 0.0f, 1.0f);
    
    if (body.bodyType == 1) {
        float halfWidth = body.dimensions.x / 2.0f;
        float halfHeight = body.dimensions.y / 2.0f;
        glBegin(GL_QUADS);
        glVertex2f(-halfWidth, -halfHeight);
        glVertex2f(halfWidth, -halfHeight);
        glVertex2f(halfWidth, halfHeight);
        glVertex2f(-halfWidth, halfHeight);
        glEnd();
    }
    else if (body.bodyType == 2) {
        float xRadius = body.dimensions.x / 2.0f;
        float yRadius = body.dimensions.y / 2.0f;
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

void PhysicsWorld::UpdatePhysics(RigidBody& body, float dt, float gravity, float dragCoeff, float boundCor) {
    body.vel.y += gravity * dt * body.gravityScale;
    body.vel.y -= dragCoeff*body.vel.y*dt/body.mass;
    body.vel.x -= dragCoeff*body.vel.x*dt/body.mass;

    if (!body.anchor)
    {
        body.pos.x += body.vel.x * dt;
        body.pos.y += body.vel.y * dt;
        body.angle += body.angularVelocity * dt;
    }

    if (body.pos.y + body.dimensions.y / 2.0f >= SCREEN_HEIGHT) {
        body.pos.y = SCREEN_HEIGHT - body.dimensions.y / 2.0f;
        body.vel.y = -body.vel.y * boundCor;
    }
    if (body.pos.x - body.dimensions.x / 2.0f <= 0.0f) {
        body.pos.x = body.dimensions.x / 2.0f;
        body.vel.x = -body.vel.x * boundCor;
    }
    if (body.pos.x + body.dimensions.x / 2.0f >= SCREEN_WIDTH) {
        body.pos.x = SCREEN_WIDTH - body.dimensions.x / 2.0f;
        body.vel.x = -body.vel.x * boundCor;
    }
    if (body.pos.y - body.dimensions.y / 2.0f <= 0.0f) {
        body.pos.y = body.dimensions.y / 2.0f;
        body.vel.y = -body.vel.y * boundCor;
    }
}

void PhysicsWorld::UpdateDistanceJoint(DistanceJoint& joint, float dt){
    RigidBody& bodyA = bodies[joint.bodyAId];
    RigidBody& bodyB = bodies[joint.bodyBId];
    float dx = bodyA.pos.x - bodyB.pos.x;
    float dy = bodyA.pos.y - bodyB.pos.y;
    float distance = pow((dx*dx)+(dy*dy), 0.5f);
    float x = distance-joint.length;
    if (abs(distance) < 0.1f) return;
    float force = -joint.strength*x;
    float bodyBaccl = -force/bodyB.mass;
    float bodyBacclX = bodyBaccl*(dx/distance);
    float bodyBacclY = bodyBaccl*(dy/distance);
    bodyB.vel.x += bodyBacclX*dt;
    bodyB.vel.y += bodyBacclY*dt;
    if (!joint.anchorA)
    {
        float bodyAaccl = force/bodyA.mass;
        float bodyAacclX = bodyAaccl*(dx/distance);
        float bodyAacclY = bodyAaccl*(dy/distance);
        bodyA.vel.x += bodyAacclX*dt;
        bodyA.vel.y += bodyAacclY*dt;
    }
}