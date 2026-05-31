#pragma once

/**
 * RigidBody.h — Rigid Body State
 *
 * Plain data structure representing a single rigid body in the simulation.
 * Position (x, y, z) and velocity (vx, vy, vz) are integrated each step.
 * Static bodies are not affected by forces or gravity.
 */
struct RigidBody {
    float x  = 0.0f, y  = 0.0f, z  = 0.0f;
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float mass    = 1.0f;
    bool  isStatic = false;
};
