#include <cmath>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include <glm/geometric.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/Solver.hpp"

namespace {

bool isFinite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

} // namespace

// This test encodes the Step 4 stability fix as a permanent regression
// guard: Step 3's naive force-based springs diverged to NaN within about
// 30 frames at any interesting stiffness (see project history). If a future
// change to Solver reintroduces that kind of instability, this test should
// catch it rather than it being noticed by eye in a debug print.
TEST_CASE("Constraint relaxation stays stable and settles over many frames", "[integrator][stability]") {
    clothsim::ClothMesh mesh(2.0f, 2.0f, /*resX=*/8, /*resY=*/8);

    for (int x = 0; x < mesh.resolutionX(); ++x) {
        clothsim::Particle& p = mesh.particles()[mesh.particleIndex(x, 0)];
        p.pinned = true;
        p.invMass = 0.0f;
    }

    clothsim::SolverParams params;
    params.constraintIterations = 5;
    params.damping = 0.96f;
    clothsim::Solver solver(params);

    const float dt = 1.0f / 60.0f;
    const int frameCount = 300;

    float maxDisplacementLastFrame = 0.0f;

    for (int frame = 0; frame < frameCount; ++frame) {
        solver.step(mesh, dt);

        float maxDisplacementThisFrame = 0.0f;
        for (const clothsim::Particle& p : mesh.particles()) {
            REQUIRE(isFinite(p.position));

            // A generous absolute bound: the cloth starts within a 2x2 area
            // near the origin, so any position with a component beyond
            // +-50 units can only mean the simulation is diverging, not
            // legitimate motion.
            REQUIRE(std::abs(p.position.x) < 50.0f);
            REQUIRE(std::abs(p.position.y) < 50.0f);
            REQUIRE(std::abs(p.position.z) < 50.0f);

            maxDisplacementThisFrame = std::max(maxDisplacementThisFrame, glm::length(p.position - p.previousPosition));
        }
        maxDisplacementLastFrame = maxDisplacementThisFrame;
    }

    // Not just "didn't explode" -- confirm it actually settled rather than
    // oscillating forever or slowly drifting (which would indicate broken
    // damping).
    REQUIRE(maxDisplacementLastFrame < 0.01f);
}
