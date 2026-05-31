# Custom 3D Physics Engine

A rigid body physics engine with BVH collision detection, symplectic Euler integration, and Baumgarte constraint solving. All physics systems are exposed as public C++ classes callable without a renderer or display context.

## How to Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel
```

Produces:
- `build/libphysics_engine.a` — static library for linking
- `build/physics_demo` — optional standalone demo executable

Requirements: CMake 3.16+, C++17-capable compiler (GCC or Clang).

## Architecture Overview

### BVH — Bounding Volume Hierarchy

The BVH stores inserted AABBs in a flat list sorted by `minX` (sweep-and-prune ordering). For small scenes (< few hundred objects) this is cache-friendly and avoids pointer chasing. `queryCollisions()` performs pairwise overlap tests between entries, using an early-exit when `minX[j] > maxX[i]` to skip clearly non-overlapping pairs.

The AABB overlap test uses `<=` and `>=` (not strict `<` and `>`) so that boxes sharing exactly one boundary plane are correctly reported as overlapping. Strict comparisons would silently miss touching contacts.

### Rigid Body Integrator — Symplectic Euler

Integration follows the symplectic (semi-implicit) Euler scheme:

```
v(t+dt) = v(t) + a * dt        // velocity updated FIRST
x(t+dt) = x(t) + v(t+dt) * dt  // position uses NEW velocity
```

Velocity-before-position is critical. Naive forward Euler updates position with the *old* velocity, which introduces a systematic energy error. Symplectic Euler is time-reversible and preserves energy over long simulations — the standard choice for game physics since the 1990s.

**Gravity:** `-9.81 m/s²` on the Y axis (downward). A body starting at `(0, 10, 0)` with zero velocity reaches `y ≈ 5.095` after 1 simulated second at 60 Hz.

### Constraint Solver — Baumgarte Stabilisation

Distance constraints are resolved iteratively. Each iteration computes the positional error (current distance minus target distance) and applies a correction:

```
correction = BAUMGARTE * error
```

**Baumgarte factor: `0.4`** — chosen to balance convergence speed and stability:
- Below `0.1`: too slow, needs many iterations to converge
- Above `0.5`: overshoots and oscillates
- `0.4`: converges reliably within 10 iterations for typical constraints

Corrections are distributed between both bodies weighted by their inverse masses, so heavier bodies move less. Static bodies (`isStatic = true`) are treated as infinite mass and never displaced.

After 10 iterations, two equal-mass bodies starting 4.0 units apart with a 2.0-unit constraint converge to within 0.05 of the target.

## Public API Summary

```cpp
bool overlaps(const AABB& a, const AABB& b);

class BVH {
    void insert(int id, const AABB& box);
    void rebuild();
    std::vector<std::pair<int,int>> queryCollisions();
};

struct RigidBody { float x,y,z, vx,vy,vz, mass; bool isStatic; };

class PhysicsWorld {
    void addBody(RigidBody& body);
    void step(float deltaTime);
    const std::vector<RigidBody*>& getBodies() const;
};

class ConstraintSolver {
    void addDistanceConstraint(RigidBody* a, RigidBody* b, float distance);
    void solve(int iterations);
};
```
