#include "distanceJoint.h"
#include "../physicsWorld.h"


DistanceJoint::DistanceJoint(float length, float strength, int bodyAId, int bodyBId, bool anchorA, PhysicsWorld* world)
: length(length), strength(strength), bodyAId(bodyAId), bodyBId(bodyBId), anchorA(anchorA), world(world){
    if (anchorA){world->SetAnchor(bodyAId, true);}
    else {world->SetAnchor(bodyAId, false);}
    world->SetAnchor(bodyBId,false);
}