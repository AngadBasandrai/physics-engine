#pragma once

enum class ShapeType {
    RECTANGLE,
    ELLIPSE32
};

struct RigidBody {
    float x, y;              
    float vx, vy;            
    float width, height;     
    float mass;        
    ShapeType shape;         
};
