#pragma once
#include "../core/rigidbody.h"
#include <cmath>

bool CheckCollisionRectRect(const RigidBody& a, const RigidBody& b) {
    float halfWidthA = a.width / 2.0f;
    float halfHeightA = a.height / 2.0f;
    float halfWidthB = b.width / 2.0f;
    float halfHeightB = b.height / 2.0f;

    return (fabs(a.x - b.x) < halfWidthA + halfWidthB) && (fabs(a.y - b.y) < halfHeightA + halfHeightB);
}

bool CheckCollisionEllipseEllipse(const RigidBody& a, const RigidBody& b) {
    float radiusA = sqrtf((a.width * a.width + a.height * a.height) / 4.0f);
    float radiusB = sqrtf((b.width * b.width + b.height * b.height) / 4.0f);
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float distSquared = dx * dx + dy * dy;
    float radiusSum = radiusA + radiusB;
    return distSquared < (radiusSum * radiusSum);
}

bool CheckCollisionRectEllipse(const RigidBody& rect, const RigidBody& ellipse) {
    float halfWidth = rect.width / 2.0f;
    float halfHeight = rect.height / 2.0f;

    float closestX = fmaxf(rect.x - halfWidth, fminf(ellipse.x, rect.x + halfWidth));
    float closestY = fmaxf(rect.y - halfHeight, fminf(ellipse.y, rect.y + halfHeight));

    float dx = ellipse.x - closestX;
    float dy = ellipse.y - closestY;
    float radiusX = ellipse.width / 2.0f;
    float radiusY = ellipse.height / 2.0f;

    return ((dx * dx) / (radiusX * radiusX) + (dy * dy) / (radiusY * radiusY)) <= 1.0f;
}

bool CheckCollision(const RigidBody& a, const RigidBody& b) {
    if (a.shape == ShapeType::RECTANGLE && b.shape == ShapeType::RECTANGLE) {
        return CheckCollisionRectRect(a, b);
    }
    else if (a.shape == ShapeType::ELLIPSE && b.shape == ShapeType::ELLIPSE) {
        return CheckCollisionEllipseEllipse(a, b);
    }
    else if ((a.shape == ShapeType::ELLIPSE && b.shape == ShapeType::RECTANGLE) || (a.shape == ShapeType::RECTANGLE && b.shape == ShapeType::ELLIPSE)) {
        const RigidBody& rect = (a.shape == ShapeType::RECTANGLE) ? a : b;
        const RigidBody& ellipse = (a.shape == ShapeType::ELLIPSE) ? a : b;
        return CheckCollisionRectEllipse(rect, ellipse);
    }
    return false;
}

void ResolveCollision(RigidBody& a, RigidBody& b, float e = 1.0f) {
    //Compute collision normal (from a to b)
    float nx = b.x - a.x;
    float ny = b.y - a.y;
    float dist = sqrt(nx * nx + ny * ny);

    if (dist == 0.0f) return;

    nx /= dist;
    ny /= dist;

    float rvx = b.vx - a.vx;
    float rvy = b.vy - a.vy;

    //Relative velocity along normal
    float velAlongNormal = rvx * nx + rvy * ny;

    // If objects are separating, no impulse
    if (velAlongNormal > 0) return;

    float invMassA = (a.mass > 0) ? 1.0f / a.mass : 0.0f;
    float invMassB = (b.mass > 0) ? 1.0f / b.mass : 0.0f;

    float j = -(1.0f + e) * velAlongNormal;
    j /= (invMassA + invMassB);

    float impulseX = j * nx;
    float impulseY = j * ny;

    a.vx -= impulseX * invMassA;
    a.vy -= impulseY * invMassA;

    b.vx += impulseX * invMassB;
    b.vy += impulseY * invMassB;
}

