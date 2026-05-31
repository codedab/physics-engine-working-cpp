#pragma once

#include "RigidBody.h"
#include <vector>

/**
 * PhysicsWorld.h — Physics Simulation World
 *
 * Manages a collection of rigid bodies and advances the simulation using
 * symplectic Euler integration with constant downward gravity (-9.81 m/s²
 * on the Y axis).
 */
class PhysicsWorld {
public:
    PhysicsWorld() = default;
    ~PhysicsWorld() = default;

    /**
     * Register a rigid body with the world.
     * The world holds a non-owning pointer; the caller must keep the body alive.
     */
    void addBody(RigidBody& body);

    /**
     * Advance the simulation by deltaTime seconds using symplectic Euler:
     *   1. Apply gravity to velocity (vy -= 9.81 * dt)
     *   2. Integrate position using the updated velocity
     * Static bodies (isStatic == true) are skipped.
     */
    void step(float deltaTime);

    /**
     * Return the list of all registered bodies.
     */
    const std::vector<RigidBody*>& getBodies() const;

private:
    std::vector<RigidBody*> _bodies;

    static constexpr float GRAVITY = 9.81f;
};
