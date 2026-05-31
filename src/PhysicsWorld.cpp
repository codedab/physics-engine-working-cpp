#include "PhysicsWorld.h"

/**
 * PhysicsWorld implementation.
 *
 * Integration uses symplectic Euler (also called semi-implicit Euler):
 *
 *   v(t+dt) = v(t) + a * dt          <- velocity updated FIRST
 *   x(t+dt) = x(t) + v(t+dt) * dt   <- position uses NEW velocity
 *
 * This ordering conserves energy better than naive forward Euler and is
 * the standard choice for real-time physics engines. For a body in free
 * fall starting at y=10, after 60 steps at dt=1/60 the analytic result
 * is y ≈ 5.095, which symplectic Euler matches within the required 0.1
 * tolerance.
 *
 * Gravity acts downward (negative Y), matching the standard right-handed
 * coordinate system used in 3D game engines.
 */

void PhysicsWorld::addBody(RigidBody& body) {
    _bodies.push_back(&body);
}

void PhysicsWorld::step(float deltaTime) {
    for (auto* body : _bodies) {
        if (body->isStatic) continue;

        // 1. Apply gravity: downward acceleration on Y axis
        body->vy -= GRAVITY * deltaTime;

        // 2. Integrate position using the UPDATED velocity (symplectic Euler)
        body->x += body->vx * deltaTime;
        body->y += body->vy * deltaTime;
        body->z += body->vz * deltaTime;
    }
}

const std::vector<RigidBody*>& PhysicsWorld::getBodies() const {
    return _bodies;
}
