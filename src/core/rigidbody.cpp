#include "rigidbody.h"
#include <cmath>
    
RigidBody::RigidBody(float x, float y, float vx, float vy, float width, float height, 
              float mass, float detail, int gravityScale, bool anchor)
        : pos(vector2(x, y)), vel(vector2(vx, vy)), dimensions(vector2(width, height)), 
          mass(mass), detail(detail), angle(0.0f), angularVelocity(0.0f), gravityScale(gravityScale){
        CalculateMomentOfInertia();
        setType();
    }
    
void RigidBody::CalculateMomentOfInertia() {
    momentOfInertia = mass * (dimensions.x*dimensions.x + dimensions.y*dimensions.y);
}

float RigidBody::calculateEnergy(float gravity){
    float vSquared = vel.x * vel.x + vel.y * vel.y;
    float kineticEnergy = 0.5f * mass * vSquared;
    
    float rotationalEnergy = 0.5f * momentOfInertia * angularVelocity * angularVelocity;
    
    float height = 72.0f - pos.y;
    float potentialEnergy = mass * gravity * height;
    energy = kineticEnergy + rotationalEnergy + potentialEnergy;
    return energy;
}

void RigidBody::setType(){
    bodyType = 0;
}

// finds the effective length for air drag
float RigidBody::effectiveLength(){
    return (dimensions.x+dimensions.y)/2;
}

void RectangularBody::CalculateMomentOfInertia() {
    // I = (1/12) * m * (w^2 + h^2)
    momentOfInertia = (mass / 12.0f) * (dimensions.x * dimensions.x + dimensions.y * dimensions.y);
}

void RectangularBody::setType(){
    bodyType = 1;
}

void EllipticalBody::CalculateMomentOfInertia() {
    // I = (1/4) * m * (a^2 + b^2)
    float a = dimensions.x / 2.0f;
    float b = dimensions.y / 2.0f;
    momentOfInertia = (mass / 4.0f) * (a * a + b * b);
}

void EllipticalBody::setType(){
    bodyType = 2;
}

float RectangularBody::effectiveLength(){
    return abs(dimensions.x*cos(angle)) + abs(dimensions.y*sin(angle));
}

float EllipticalBody::effectiveLength(){
    return pow((dimensions.x*dimensions.x*cos(angle)*cos(angle))+(dimensions.y*dimensions.y*sin(angle)*sin(angle)), 0.5f);
}