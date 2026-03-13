#pragma once

class PhysicsWorld;
struct vector2;

struct DistanceJoint {
    float length;
    float strength;
    int bodyAId;
    int bodyBId;
    bool anchorA;
    PhysicsWorld* world;
    float energy; 
    
    DistanceJoint(float length, float strength, int bodyAId, int bodyBId, bool anchorA, PhysicsWorld* world);
    float calculateEnergy(vector2 pos1, vector2 pos2);
};