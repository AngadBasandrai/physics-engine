#pragma once

class PhysicsWorld;

struct DistanceJoint {
    float length;
    float strength;
    int bodyAId;
    int bodyBId;
    bool anchorA;
    PhysicsWorld* world;

    DistanceJoint(float length, float strength, int bodyAId, int bodyBId, bool anchorA, PhysicsWorld* world);

};