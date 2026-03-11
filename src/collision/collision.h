#pragma once
#include "../core/rigidbody.h"
#include "../constants.h"

struct ContactPoint {
    float x, y;
    float nx, ny;
    float penetration;
};

void RotatePoint(float& px, float& py, float angle);

void GetRectCorners(const RigidBody& rect, float corners[4][2]);

void ProjectRectangle(const RigidBody& rect, float axisX, float axisY, float& min, float& max);

bool CheckCollisionRectRect(const RigidBody& a, const RigidBody& b, ContactPoint* contact = nullptr);

bool CheckCollisionEllipseEllipse(const RigidBody& a, const RigidBody& b, ContactPoint* contact = nullptr);

bool CheckCollisionRectEllipse(const RigidBody& rect, const RigidBody& ellipse, ContactPoint* contact = nullptr);

bool CheckCollision(const RigidBody& a, const RigidBody& b, ContactPoint* contact = nullptr);

void ResolveCollision(RigidBody& a, RigidBody& b, const ContactPoint& contact, float e);