#pragma once

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

#include "clothsim/Particle.hpp"

namespace clothsim {

// Integer coordinate of a grid cell.
struct CellKey {
    int x;
    int y;
    int z;

    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const {
        // Cheap 3-way hash combine; good enough for a spatial-hash bucket
        // key, not a cryptographic hash.
        std::size_t h = std::hash<int>()(key.x);
        h ^= std::hash<int>()(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

// Uniform spatial hash grid: buckets particles into fixed-size cells keyed
// by integer coordinates, so "which particles are near this one" can be
// answered by looking at a fixed 27-cell neighborhood instead of scanning
// every particle. Rebuilt from scratch each time build() is called, since
// particle positions change every frame.
//
// Complexity: a brute-force all-pairs check is O(n^2) -- every particle
// tested against every other. With particles spread roughly evenly through
// space, each cell holds a small, roughly constant number of particles
// regardless of n, so build() is O(n) and forEachCandidatePair() visits
// O(n) candidate pairs in total rather than O(n^2). The trade is that it's
// an approximation: cellSize must be >= the distance you care about, or
// candidates in range could land in a cell outside the 27 searched.
class SpatialHashGrid {
public:
    explicit SpatialHashGrid(float cellSize);

    // Buckets every particle into its cell. Call once per frame before
    // querying, since positions have moved since the last build.
    void build(const std::vector<Particle>& particles);

    // Invokes func(i, j) with i < j exactly once for every unique pair of
    // particle indices whose cells are the same or directly adjacent
    // (26-neighborhood + own cell). Candidates only -- callers must still
    // check actual distance, since two particles in adjacent cells are not
    // necessarily within cellSize of each other.
    template <typename Func>
    void forEachCandidatePair(Func&& func) const {
        for (const auto& [key, indices] : m_cells) {
            for (std::size_t i : indices) {
                for (int dz = -1; dz <= 1; ++dz) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            const CellKey neighborKey{key.x + dx, key.y + dy, key.z + dz};
                            const auto it = m_cells.find(neighborKey);
                            if (it == m_cells.end()) {
                                continue;
                            }
                            for (std::size_t j : it->second) {
                                if (j > i) {
                                    func(i, j);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

private:
    float m_cellSize;
    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> m_cells;

    CellKey cellKeyFor(const glm::vec3& position) const;
};

} // namespace clothsim
