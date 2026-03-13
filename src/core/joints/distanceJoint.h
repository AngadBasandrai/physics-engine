#pragma once

class PhysicsWorld;
struct vector2;

struct DistanceJoint {
public:
    bool anchorA;
    
    DistanceJoint(float length, float strength, int bodyAId, int bodyBId, bool anchorA, PhysicsWorld* world);
    float calculateEnergy(vector2 pos1, vector2 pos2);
    vector2 getId();
    float getStrength();
    float getLength();
    
private:
    PhysicsWorld* world;
    float length;
    float strength;
    int bodyAId;
    int bodyBId;
    float energy; 
};