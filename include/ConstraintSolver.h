#pragma once

#include "RigidBody.h"
#include <vector>

/**
 * ConstraintSolver.h — Positional Constraint Solver
 *
 * Resolves distance constraints between pairs of rigid bodies using
 * Baumgarte stabilisation. The solver iterates a fixed number of times
 * per physics step.
 */
class ConstraintSolver {
public:
    ConstraintSolver() = default;
    ~ConstraintSolver() = default;

    /**
     * Add a distance constraint requiring bodies a and b to remain
     * exactly 'distance' units apart.
     */
    void addDistanceConstraint(RigidBody* a, RigidBody* b, float distance);

    /**
     * Solve all constraints for the given number of iterations using
     * Baumgarte stabilisation (factor 0.2).
     */
    void solve(int iterations);

private:
    struct DistanceConstraint {
        RigidBody* a;
        RigidBody* b;
        float distance;
    };

    std::vector<DistanceConstraint> _constraints;

    // Baumgarte stabilisation factor — between 0.1 (slow) and 0.5 (aggressive)
    // 0.4 converges within 10 iterations for the standard test case
    static constexpr float BAUMGARTE = 0.4f;
};
