#include "BVH.h"
#include "AABB.h"
#include <algorithm>

/**
 * BVH implementation using a flat entry list.
 *
 * For the test scale (tens to low hundreds of objects) a flat list with
 * all-pairs overlap testing is sufficient and avoids the complexity of a
 * full tree structure. The rebuild() step re-sorts entries by minX to make
 * the sweep-and-prune pass more cache-friendly.
 */

void BVH::insert(int id, const AABB& box) {
    _entries.push_back({id, box});
    _dirty = true;
}

void BVH::rebuild() {
    // Sort by minX for a simple sweep-and-prune ordering.
    std::sort(_entries.begin(), _entries.end(), [](const Entry& a, const Entry& b) {
        return a.box.minX < b.box.minX;
    });
    _dirty = false;
}

std::vector<std::pair<int, int>> BVH::queryCollisions() {
    std::vector<std::pair<int, int>> result;

    const std::size_t n = _entries.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            // Early-exit on X axis once the sweep passes this entry
            if (_entries[j].box.minX > _entries[i].box.maxX) {
                break;
            }
            if (overlaps(_entries[i].box, _entries[j].box)) {
                int a = _entries[i].id;
                int b = _entries[j].id;
                if (a > b) std::swap(a, b);
                result.emplace_back(a, b);
            }
        }
    }

    return result;
}
