#include <iostream>
#include <cmath>
#include "AABB.h"
#include "BVH.h"
#include "RigidBody.h"
#include "PhysicsWorld.h"
#include "ConstraintSolver.h"

int main() {
    std::cout << "=== Custom 3D Physics Engine Demo ===\n\n";

    // ---- BVH collision detection ----
    std::cout << "[BVH] Inserting two overlapping AABBs...\n";
    BVH bvh;
    AABB boxA{0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
    AABB boxB{0.5f, 0.0f, 0.0f, 1.5f, 1.0f, 1.0f};  // overlaps boxA by 0.5 on X
    bvh.insert(0, boxA);
    bvh.insert(1, boxB);
    bvh.rebuild();
    auto pairs = bvh.queryCollisions();
    std::cout << "[BVH] Collision pairs found: " << pairs.size() << "\n";
    for (auto& [a, b] : pairs) {
        std::cout << "      Pair (" << a << ", " << b << ")\n";
    }

    // ---- Rigid body free-fall ----
    std::cout << "\n[PhysicsWorld] Free-fall test: body at (0, 10, 0) for 1 second at 60Hz...\n";
    PhysicsWorld world;
    RigidBody body;
    body.x = 0.0f; body.y = 10.0f; body.z = 0.0f;
    body.vx = 0.0f; body.vy = 0.0f; body.vz = 0.0f;
    body.mass = 1.0f; body.isStatic = false;
    world.addBody(body);

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) {
        world.step(dt);
    }
    std::cout << "[PhysicsWorld] Final Y position: " << body.y
              << " (expected ~5.095, tolerance ±0.1)\n";
    bool passIntegration = std::abs(body.y - 5.095f) <= 0.1f;
    std::cout << "[PhysicsWorld] Integration test: " << (passIntegration ? "PASS" : "FAIL") << "\n";

    // ---- Constraint solver ----
    std::cout << "\n[ConstraintSolver] Distance constraint: target 2.0 units, start 4.0 units apart...\n";
    RigidBody bodyA, bodyB;
    bodyA.x = 0.0f; bodyA.y = 0.0f; bodyA.z = 0.0f; bodyA.mass = 1.0f;
    bodyB.x = 4.0f; bodyB.y = 0.0f; bodyB.z = 0.0f; bodyB.mass = 1.0f;

    ConstraintSolver solver;
    solver.addDistanceConstraint(&bodyA, &bodyB, 2.0f);
    solver.solve(10);

    float dx = bodyB.x - bodyA.x;
    float dy = bodyB.y - bodyA.y;
    float dz = bodyB.z - bodyA.z;
    float finalDist = std::sqrt(dx*dx + dy*dy + dz*dz);
    std::cout << "[ConstraintSolver] Final distance: " << finalDist
              << " (expected ~2.0, tolerance ±0.05)\n";
    bool passConstraint = std::abs(finalDist - 2.0f) <= 0.05f;
    std::cout << "[ConstraintSolver] Constraint test: " << (passConstraint ? "PASS" : "FAIL") << "\n";

    std::cout << "\n=== Demo complete ===\n";
    return (passIntegration && passConstraint) ? 0 : 1;
}
