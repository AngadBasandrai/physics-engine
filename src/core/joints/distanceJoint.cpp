#include "distanceJoint.h"
#include "../physicsWorld.h"
#include "../structures/structures.h"
#include <cmath>


DistanceJoint::DistanceJoint(float length, float strength, int bodyAId, int bodyBId, bool anchorA, PhysicsWorld* world)
: length(length), strength(strength), bodyAId(bodyAId), bodyBId(bodyBId), anchorA(anchorA), world(world){
    if (anchorA){world->SetAnchor(bodyAId, true);}
    else {world->SetAnchor(bodyAId, false);}
    world->SetAnchor(bodyBId,false);
}

float DistanceJoint::calculateEnergy(vector2 pos1, vector2 pos2){
    vector2 del = pos1 - pos2;
    float dist = pow(del.x*del.x + del.y*del.y, 0.5f);
    float x = abs(length-dist);
    energy = strength*x*x/2.0f;
    return energy;
}