#pragma once

enum class ShapeType {
    RECTANGLE,
    ELLIPSE
};

struct RigidBody {
    float x, y;              
    float vx, vy;            
    float width, height;     
    float mass;        
    ShapeType shape;         
    float detail = 32;
};
