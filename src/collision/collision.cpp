#include "collision.h"
#include <cmath>
#include <vector>

void RotatePoint(float& px, float& py, float angle) {
    float c = cosf(angle);
    float s = sinf(angle);
    float newX = px * c - py * s;
    float newY = px * s + py * c;
    px = newX;
    py = newY;
}

void GetRectCorners(const RigidBody& rect, float corners[4][2]) {
    float halfW = rect.width / 2.0f;
    float halfH = rect.height / 2.0f;
    
    corners[0][0] = -halfW; corners[0][1] = -halfH;
    corners[1][0] =  halfW; corners[1][1] = -halfH;
    corners[2][0] =  halfW; corners[2][1] =  halfH;
    corners[3][0] = -halfW; corners[3][1] =  halfH;
    
    for (int i = 0; i < 4; i++) {
        RotatePoint(corners[i][0], corners[i][1], rect.angle);
        corners[i][0] += rect.x;
        corners[i][1] += rect.y;
    }
}

void ProjectRectangle(const RigidBody& rect, float axisX, float axisY, float& min, float& max) {
    float corners[4][2];
    GetRectCorners(rect, corners);
    
    min = max = corners[0][0] * axisX + corners[0][1] * axisY;
    
    for (int i = 1; i < 4; i++) {
        float projection = corners[i][0] * axisX + corners[i][1] * axisY;
        if (projection < min) min = projection;
        if (projection > max) max = projection;
    }
}

bool CheckCollisionRectRect(const RigidBody& a, const RigidBody& b, ContactPoint* contact) {
    float axes[4][2];
    
    axes[0][0] = cosf(a.angle); axes[0][1] = sinf(a.angle);
    axes[1][0] = -sinf(a.angle); axes[1][1] = cosf(a.angle);
    
    axes[2][0] = cosf(b.angle); axes[2][1] = sinf(b.angle);
    axes[3][0] = -sinf(b.angle); axes[3][1] = cosf(b.angle);
    
    float minOverlap = INFINITY;
    float bestAxisX = 0, bestAxisY = 0;
    
    for (int i = 0; i < 4; i++) {
        float axisX = axes[i][0];
        float axisY = axes[i][1];
        
        float minA, maxA, minB, maxB;
        ProjectRectangle(a, axisX, axisY, minA, maxA);
        ProjectRectangle(b, axisX, axisY, minB, maxB);
        
        if (maxA < minB || maxB < minA) {
            return false;
        }
        
        float overlap = fminf(maxA, maxB) - fmaxf(minA, minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxisX = axisX;
            bestAxisY = axisY;
        }
    }
    
    if (contact) {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        if (dx * bestAxisX + dy * bestAxisY < 0) {
            bestAxisX = -bestAxisX;
            bestAxisY = -bestAxisY;
        }
        
        contact->nx = bestAxisX;
        contact->ny = bestAxisY;
        contact->penetration = minOverlap;
        contact->x = (a.x + b.x) / 2.0f;
        contact->y = (a.y + b.y) / 2.0f;
    }
    
    return true;
}

bool CheckCollisionEllipseEllipse(const RigidBody& a, const RigidBody& b, ContactPoint* contact) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    
    float ca = cosf(-a.angle);
    float sa = sinf(-a.angle);
    float localBx = dx * ca - dy * sa;
    float localBy = dx * sa + dy * ca;
    
    float relativeAngle = b.angle - a.angle;
    
    float ax = a.width / 2.0f;
    float ay = a.height / 2.0f;
    float bx = b.width / 2.0f;
    float by = b.height / 2.0f;
    
    float dist = sqrtf(localBx * localBx + localBy * localBy);
    
    float angle = atan2f(localBy, localBx);
    float radiusA = (ax * ay) / sqrtf((ay * cosf(angle)) * (ay * cosf(angle)) + 
                                       (ax * sinf(angle)) * (ax * sinf(angle)));
    
    float radiusB = sqrtf(bx * bx + by * by);
    
    if (dist < radiusA + radiusB) {
        if (contact && dist > 0.0001f) {
            float localNx = localBx / dist;
            float localNy = localBy / dist;
            
            contact->nx = localNx * ca + localNy * sa;
            contact->ny = -localNx * sa + localNy * ca;
            contact->penetration = radiusA + radiusB - dist;
            contact->x = a.x + contact->nx * radiusA;
            contact->y = a.y + contact->ny * radiusA;
        }
        return true;
    }
    return false;
}

bool CheckCollisionRectEllipse(const RigidBody& rect, const RigidBody& ellipse, ContactPoint* contact) {
    float localX = ellipse.x - rect.x;
    float localY = ellipse.y - rect.y;
    
    float c = cosf(-rect.angle);
    float s = sinf(-rect.angle);
    float rotatedX = localX * c - localY * s;
    float rotatedY = localX * s + localY * c;
    
    float relativeAngle = ellipse.angle - rect.angle;
    
    float halfWidth = rect.width / 2.0f;
    float halfHeight = rect.height / 2.0f;
    
    float closestX = fmaxf(-halfWidth, fminf(rotatedX, halfWidth));
    float closestY = fmaxf(-halfHeight, fminf(rotatedY, halfHeight));
    
    float dx = rotatedX - closestX;
    float dy = rotatedY - closestY;
    
    float ce = cosf(-relativeAngle);
    float se = sinf(-relativeAngle);
    float ellipseLocalX = dx * ce - dy * se;
    float ellipseLocalY = dx * se + dy * ce;
    
    float radiusX = ellipse.width / 2.0f;
    float radiusY = ellipse.height / 2.0f;
    
    bool collision = ((ellipseLocalX * ellipseLocalX) / (radiusX * radiusX) + 
                     (ellipseLocalY * ellipseLocalY) / (radiusY * radiusY)) <= 1.0f;
    
    if (collision && contact) {
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > 0.0001f) {
            float localNx = dx / dist;
            float localNy = dy / dist;
            
            contact->nx = localNx * c + localNy * s;
            contact->ny = -localNx * s + localNy * c;
            
            // Approximate penetration
            float ellipseRadiusInDir = (radiusX * radiusY) / 
                sqrtf((radiusY * ce * ellipseLocalX / radiusX) * (radiusY * ce * ellipseLocalX / radiusX) + 
                      (radiusX * se * ellipseLocalY / radiusY) * (radiusX * se * ellipseLocalY / radiusY) + 0.0001f);
            contact->penetration = ellipseRadiusInDir - dist;
            
            contact->x = closestX * c + closestY * s + rect.x;
            contact->y = -closestX * s + closestY * c + rect.y;
        }
    }
    
    return collision;
}

bool CheckCollision(const RigidBody& a, const RigidBody& b, ContactPoint* contact) {
    if (a.shape == ShapeType::RECTANGLE && b.shape == ShapeType::RECTANGLE) {
        return CheckCollisionRectRect(a, b, contact);
    }
    else if (a.shape == ShapeType::ELLIPSE && b.shape == ShapeType::ELLIPSE) {
        return CheckCollisionEllipseEllipse(a, b, contact);
    }
    else if ((a.shape == ShapeType::ELLIPSE && b.shape == ShapeType::RECTANGLE) || 
             (a.shape == ShapeType::RECTANGLE && b.shape == ShapeType::ELLIPSE)) {
        const RigidBody& rect = (a.shape == ShapeType::RECTANGLE) ? a : b;
        const RigidBody& ellipse = (a.shape == ShapeType::ELLIPSE) ? a : b;
        bool result = CheckCollisionRectEllipse(rect, ellipse, contact);
        
        // Flip normal if order was reversed
        if (contact && a.shape == ShapeType::ELLIPSE) {
            contact->nx = -contact->nx;
            contact->ny = -contact->ny;
        }
        return result;
    }
    return false;
}

void ResolveCollision(RigidBody& a, RigidBody& b, const ContactPoint& contact, float e) {
    float nx = contact.nx;
    float ny = contact.ny;
    float rax = contact.x - a.x;
    float ray = contact.y - a.y;
    float rbx = contact.x - b.x;
    float rby = contact.y - b.y;

    float vax = a.vx - a.angularVelocity * ray;
    float vay = a.vy + a.angularVelocity * rax;
    float vbx = b.vx - b.angularVelocity * rby;
    float vby = b.vy + b.angularVelocity * rbx;
    float rvx = vbx - vax;
    float rvy = vby - vay;

    float velAlongNormal = rvx * nx + rvy * ny;
    if (velAlongNormal > 0) return;

    float invMassA = (a.mass > 0) ? 1.0f / a.mass : 0.0f;
    float invMassB = (b.mass > 0) ? 1.0f / b.mass : 0.0f;
    float invInertiaA = (a.momentOfInertia > 0) ? 1.0f / a.momentOfInertia : 0.0f;
    float invInertiaB = (b.momentOfInertia > 0) ? 1.0f / b.momentOfInertia : 0.0f;

    float raCrossN = rax * ny - ray * nx;
    float rbCrossN = rbx * ny - rby * nx;
    float denominator = invMassA + invMassB + raCrossN * raCrossN * invInertiaA + rbCrossN * rbCrossN * invInertiaB;

    float j = -(1.0f + e) * velAlongNormal / denominator;
    float impulseX = j * nx;
    float impulseY = j * ny;

    a.vx -= impulseX * invMassA;
    a.vy -= impulseY * invMassA;
    b.vx += impulseX * invMassB;
    b.vy += impulseY * invMassB;
    a.angularVelocity -= (rax * impulseY - ray * impulseX) * invInertiaA;
    b.angularVelocity += (rbx * impulseY - rby * impulseX) * invInertiaB;

    if (STATIC_FRICTION > 0 || DYNAMIC_FRICTION > 0) {
        float tx = -ny;
        float ty = nx;
        float velAlongTangent = rvx * tx + rvy * ty;

        float raCrossT = rax * ty - ray * tx;
        float rbCrossT = rbx * ty - rby * tx;
        float tangentDenom = invMassA + invMassB + raCrossT * raCrossT * invInertiaA + rbCrossT * rbCrossT * invInertiaB;

        float jt = -velAlongTangent / tangentDenom;

        float frictionImpulse;
        if (fabs(jt) < j * STATIC_FRICTION)
            frictionImpulse = jt;
        else
            frictionImpulse = -j * DYNAMIC_FRICTION * ((jt < 0) ? -1.0f : 1.0f);

        float frictionX = frictionImpulse * tx;
        float frictionY = frictionImpulse * ty;

        a.vx -= frictionX * invMassA;
        a.vy -= frictionY * invMassA;
        b.vx += frictionX * invMassB;
        b.vy += frictionY * invMassB;
        a.angularVelocity -= (rax * frictionY - ray * frictionX) * invInertiaA;
        b.angularVelocity += (rbx * frictionY - rby * frictionX) * invInertiaB;
    }
}
