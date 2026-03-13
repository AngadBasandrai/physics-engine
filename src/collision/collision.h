#pragma once
#include "../core/rigidbody.h"
#include "../constants.h"

struct ContactPoint {
    float x, y;
    float nx, ny;
    float penetration;
};

void RotatePoint(float& px, float& py, float angle);

void GetRectCorners(RigidBody& rect, float corners[4][2]);

void ProjectRectangle(RigidBody& rect, float axisX, float axisY, float& min, float& max);

bool CheckCollisionRectRect(RigidBody& a, RigidBody& b, ContactPoint* contact = nullptr);

bool CheckCollisionEllipseEllipse(RigidBody& a, RigidBody& b, ContactPoint* contact = nullptr);

bool CheckCollisionRectEllipse(RigidBody& rect, RigidBody& ellipse, ContactPoint* contact = nullptr);

bool CheckCollision(RigidBody& a, RigidBody& b, ContactPoint* contact = nullptr);

void ResolveCollision(RigidBody& a, RigidBody& b, const ContactPoint& contact, float e);