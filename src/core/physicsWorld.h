#pragma once

#include "rigidbody.h"
#include "../collision/collision.h"
#include <vector>
#include <GLFW/glfw3.h>
#include <cmath>

struct DistanceJoint;

class PhysicsWorld {
public:
    float energy;

    PhysicsWorld();
    ~PhysicsWorld();

    int AddBody(const RigidBody& body);
    void AddDistanceJoint(const DistanceJoint& joint);
    void SetAnchor(int id, bool anchor = true);
    void Render();
    float CalculateTotalEnergy(float gravity);
    void Update(float dt, float gravity, float viscosity, float boundCor);
    
private:
    std::vector<RigidBody> bodies;
    std::vector<DistanceJoint> distanceJoints;
    
    void RenderBody(RigidBody& body);
    void UpdatePhysics(RigidBody& body, float dt, float gravity, float viscosity, float boundCor);
    void UpdateDistanceJoint(DistanceJoint& joint, float dt);

};