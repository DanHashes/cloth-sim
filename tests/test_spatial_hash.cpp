#include <cstddef>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include <glm/geometric.hpp>

#include "clothsim/Particle.hpp"
#include "clothsim/SpatialHashGrid.hpp"

// SpatialHashGrid::forEachCandidatePair() is deliberately an over-approximation:
// two particles in neighboring cells are not necessarily within cellSize of
// each other, so candidates are a *superset* of the true neighbor pairs (see
// SpatialHashGrid.hpp). The correctness property that actually matters for
// self-collision is completeness -- no true neighbor pair may ever be
// missed -- not that the candidate set is exactly equal to the brute-force
// set. This test checks exactly that against a random particle cloud with a
// fixed seed, so it's fully deterministic.
TEST_CASE("SpatialHashGrid never misses a pair the brute-force check finds", "[spatial_hash]") {
    std::mt19937 rng(42); // fixed seed: deterministic test, no flakiness
    std::uniform_real_distribution<float> dist(-2.0f, 2.0f);

    std::vector<clothsim::Particle> particles(80);
    for (clothsim::Particle& p : particles) {
        p.position = glm::vec3(dist(rng), dist(rng), dist(rng));
    }

    const float cellSize = 0.5f;

    std::set<std::pair<std::size_t, std::size_t>> bruteForcePairs;
    for (std::size_t i = 0; i < particles.size(); ++i) {
        for (std::size_t j = i + 1; j < particles.size(); ++j) {
            if (glm::length(particles[i].position - particles[j].position) < cellSize) {
                bruteForcePairs.emplace(i, j);
            }
        }
    }
    REQUIRE(bruteForcePairs.size() > 0); // sanity: the test setup actually has neighbors to find

    clothsim::SpatialHashGrid grid(cellSize);
    grid.build(particles);

    std::set<std::pair<std::size_t, std::size_t>> candidatePairs;
    grid.forEachCandidatePair([&](std::size_t i, std::size_t j) {
        candidatePairs.emplace(i, j);
    });

    for (const auto& pair : bruteForcePairs) {
        INFO("brute-force pair (" << pair.first << ", " << pair.second << ") missing from grid candidates");
        REQUIRE(candidatePairs.count(pair) == 1);
    }
}

TEST_CASE("SpatialHashGrid reports each candidate pair exactly once", "[spatial_hash]") {
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    std::vector<clothsim::Particle> particles(40);
    for (clothsim::Particle& p : particles) {
        p.position = glm::vec3(dist(rng), dist(rng), dist(rng));
    }

    clothsim::SpatialHashGrid grid(0.4f);
    grid.build(particles);

    std::vector<std::pair<std::size_t, std::size_t>> allPairs;
    grid.forEachCandidatePair([&](std::size_t i, std::size_t j) {
        REQUIRE(i < j); // contract: always reported in (smaller, larger) order
        allPairs.emplace_back(i, j);
    });

    const std::set<std::pair<std::size_t, std::size_t>> uniquePairs(allPairs.begin(), allPairs.end());
    REQUIRE(uniquePairs.size() == allPairs.size());
}
