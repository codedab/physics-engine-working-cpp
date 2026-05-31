#pragma once

/**
 * AABB.h — Axis-Aligned Bounding Box
 *
 * Defines the AABB structure and the overlap test used by the BVH for
 * broad-phase collision detection.
 */

struct AABB {
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
};

/**
 * Returns true if two AABBs overlap (including touching boundaries).
 * Uses <= and >= to correctly detect touching/boundary contact.
 */
bool overlaps(const AABB& a, const AABB& b);
