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
    float angle = 0.0f;           
    float angularVelocity = 0.0f;
    float momentOfInertia;       
    
    RigidBody() : momentOfInertia(0.0f) {}
    
    RigidBody(float x, float y, float vx, float vy, float width, float height, 
              float mass, ShapeType shape, float detail = 32)
        : x(x), y(y), vx(vx), vy(vy), width(width), height(height), 
          mass(mass), shape(shape), detail(detail), angle(0.0f), angularVelocity(0.0f) {
        CalculateMomentOfInertia();
    }
    
    void CalculateMomentOfInertia() {
        if (shape == ShapeType::RECTANGLE) {
            // I = (1/12) * m * (w^2 + h^2)
            momentOfInertia = (mass / 12.0f) * (width * width + height * height);
        } else if (shape == ShapeType::ELLIPSE) {
            // I = (1/4) * m * (a^2 + b^2) for ellipse
            float a = width / 2.0f;
            float b = height / 2.0f;
            momentOfInertia = (mass / 4.0f) * (a * a + b * b);
        }
    }
};