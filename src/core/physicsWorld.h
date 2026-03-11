#pragma once

#include "rigidbody.h"
#include "../collision/collision.h"
#include <vector>
#include <GLFW/glfw3.h>
#include <cmath>

struct DistanceJoint;

class PhysicsWorld {
public:
    std::vector<RigidBody> bodies;
    std::vector<DistanceJoint> distanceJoints;

    PhysicsWorld();
    ~PhysicsWorld();

    int AddBody(const RigidBody& body);
    void AddDistanceJoint(const DistanceJoint& joint);
    void Update(float dt, float gravity, float viscosity, float boundCor);
    void Render();
    void SetAnchor(int id, bool anchor = true);
    float CalculateTotalEnergy(float gravity);

private:
    void RenderBody(const RigidBody& body);
    void UpdatePhysics(RigidBody& body, float dt, float gravity, float viscosity, float boundCor);
    void UpdateDistanceJoint(DistanceJoint& joint, float dt);

};