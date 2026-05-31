#pragma once

#include "AABB.h"
#include <vector>
#include <utility>

/**
 * BVH.h — Bounding Volume Hierarchy
 *
 * Broad-phase collision detection using a flat-list BVH suitable for
 * small-to-medium scenes. Supports insertion, rebuild, and pair-wise
 * collision queries.
 */
class BVH {
public:
    BVH() = default;
    ~BVH() = default;

    /**
     * Insert an object with the given id and bounding box.
     * Multiple calls accumulate entries; call rebuild() after bulk insertions.
     */
    void insert(int id, const AABB& box);

    /**
     * Rebuild the internal acceleration structure.
     * Must be called after insertions before queryCollisions() is reliable.
     */
    void rebuild();

    /**
     * Return all overlapping pairs as (id_a, id_b) where id_a < id_b.
     */
    std::vector<std::pair<int, int>> queryCollisions();

private:
    struct Entry {
        int id;
        AABB box;
    };

    std::vector<Entry> _entries;
    bool _dirty = false;
};
