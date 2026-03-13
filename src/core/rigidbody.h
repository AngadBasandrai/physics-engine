#pragma once

#include "structures/structures.h"

class RigidBody {
public:
    vector2 pos;
    vector2 vel;
    float angle = 0.0f;           
    float angularVelocity = 0.0f;
    float momentOfInertia;
    int id; 
    int gravityScale;
    bool anchor;
    
    RigidBody(float x, float y, float vx, float vy, float width, float height, 
        float mass, float detail = 32, int gravityScale = 1, bool anchor = false);
    // finds the effective length for air drag
    virtual float effectiveLength();
    float calculateEnergy(float gravity);
    int getType();
    vector2 getDimensions();
    float getMass();
    float getDetail();

protected:
    int bodyType;
    vector2 dimensions;     
    float mass;              
    float detail = 32;
    
private:
    float energy;
    virtual void CalculateMomentOfInertia();
    virtual void setType();

};

class RectangularBody : public RigidBody{
public:
    RectangularBody(float x, float y, float vx, float vy, float width, float height, 
        float mass, float detail = 32, int gravityScale = 1, bool anchor = false);
    float effectiveLength() override;

private:
    void CalculateMomentOfInertia() override;
    void setType() override;
};

class EllipticalBody : public RigidBody{
public:
    EllipticalBody(float x, float y, float vx, float vy, float width, float height, 
        float mass, float detail = 32, int gravityScale = 1, bool anchor = false);
    float effectiveLength() override;

private:
    void CalculateMomentOfInertia() override;
    void setType() override;
};