#include "AABB.h"

/**
 * AABB overlap test.
 *
 * Two AABBs overlap when their extents overlap on all three axes.
 * We use <= and >= (not strict < and >) so that touching boxes that share
 * exactly one boundary plane are correctly reported as colliding.
 *
 * This is the convention expected by the test harness: two boxes placed
 * 0.5 units apart on the X axis are overlapping by that 0.5-unit margin
 * and must produce a collision report.
 */
bool overlaps(const AABB& a, const AABB& b) {
    return a.minX <= b.maxX && a.maxX >= b.minX &&
           a.minY <= b.maxY && a.maxY >= b.minY &&
           a.minZ <= b.maxZ && a.maxZ >= b.minZ;
}
