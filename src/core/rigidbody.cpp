#include "rigidbody.h"
#include <cmath>
    
RigidBody::RigidBody(float x, float y, float vx, float vy, float width, float height, 
              float mass, float detail, int gravityScale, bool anchor)
        : x(x), y(y), vx(vx), vy(vy), width(width), height(height), 
          mass(mass), detail(detail), angle(0.0f), angularVelocity(0.0f), gravityScale(gravityScale){
        CalculateMomentOfInertia();
        setType();
    }
    
void RigidBody::CalculateMomentOfInertia() {
    momentOfInertia = mass * (width*width + height*height);
}

void RigidBody::setType(){
    bodyType = 0;
}

// finds the effective length for air drag
float RigidBody::effectiveLength(){
    return (width+height)/2;
}

void RectangularBody::CalculateMomentOfInertia() {
    // I = (1/12) * m * (w^2 + h^2)
    momentOfInertia = (mass / 12.0f) * (width * width + height * height);
}

void RectangularBody::setType(){
    bodyType = 1;
}

void EllipticalBody::CalculateMomentOfInertia() {
    // I = (1/4) * m * (a^2 + b^2)
    float a = width / 2.0f;
    float b = height / 2.0f;
    momentOfInertia = (mass / 4.0f) * (a * a + b * b);
}

void EllipticalBody::setType(){
    bodyType = 2;
}

float RectangularBody::effectiveLength(){
    return abs(width*cos(angle)) + abs(height*sin(angle));
}

float EllipticalBody::effectiveLength(){
    return pow((width*width*cos(angle)*cos(angle))+(height*height*sin(angle)*sin(angle)), 0.5f);
}