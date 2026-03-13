#pragma once

#include "structures/structures.h"

class RigidBody {
public:
    vector2 pos;
    vector2 vel;            
    vector2 dimensions;     
    float mass;              
    float detail = 32;
    float angle = 0.0f;           
    float angularVelocity = 0.0f;
    float momentOfInertia;
    int id; 
    int gravityScale;
    bool anchor;
    int bodyType;
    float energy;
    
    RigidBody(float x, float y, float vx, float vy, float width, float height, 
              float mass, float detail = 32, int gravityScale = 1, bool anchor = false);
    virtual void CalculateMomentOfInertia();
    // finds the effective length for air drag
    virtual float effectiveLength();
    virtual void setType();
    float calculateEnergy(float gravity);
};

class RectangularBody : public RigidBody{
    using RigidBody::RigidBody;
    void CalculateMomentOfInertia() override;
    float effectiveLength() override;
    void setType() override;
};

class EllipticalBody : public RigidBody{
    using RigidBody::RigidBody;
    void CalculateMomentOfInertia() override;
    float effectiveLength() override;
    void setType() override;
};