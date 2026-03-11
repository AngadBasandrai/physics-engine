#pragma once

class RigidBody {
public:
    float x, y;
    float vx, vy;            
    float width, height;     
    float mass;              
    float detail = 32;
    float angle = 0.0f;           
    float angularVelocity = 0.0f;
    float momentOfInertia;
    int id; 
    int gravityScale;
    bool anchor;
    int bodyType;
    
    RigidBody(float x, float y, float vx, float vy, float width, float height, 
              float mass, float detail = 32, int gravityScale = 1, bool anchor = false);
    virtual void CalculateMomentOfInertia();
    // finds the effective length for air drag
    virtual float effectiveLength();
    virtual void setType();
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