#include <cstddef>
#include <fstream>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/Solver.hpp"

using Catch::Approx;

namespace {

// CLOTHSIM_BASELINES_DIR is injected by tests/CMakeLists.txt as an absolute
// path, so this test finds its baseline regardless of ctest's working
// directory.
std::string baselinePath() {
    return std::string(CLOTHSIM_BASELINES_DIR) + "/falling_cloth_baseline.txt";
}

} // namespace

// Fixed scenario, fully deterministic (no randomness, no self-collision so
// no dependence on unordered_map iteration order): a 4x4 grid with its top
// row pinned, falling under gravity for exactly 100 frames. If a future
// change to ClothMesh/Solver alters the simulation's numeric behavior --
// intentionally or not -- this test will fail and force a conscious
// decision to update the baseline, rather than the drift going unnoticed.
TEST_CASE("Falling cloth scenario matches its stored baseline", "[regression]") {
    clothsim::ClothMesh mesh(1.0f, 1.0f, 4, 4);

    for (int x = 0; x < mesh.resolutionX(); ++x) {
        clothsim::Particle& p = mesh.particles()[mesh.particleIndex(x, 0)];
        p.pinned = true;
        p.invMass = 0.0f;
    }

    clothsim::SolverParams params;
    params.gravity = glm::vec3(0.0f, -9.81f, 0.0f);
    params.damping = 0.98f;
    params.constraintIterations = 5;
    clothsim::Solver solver(params);

    const float dt = 1.0f / 60.0f;
    const int frameCount = 100;
    for (int frame = 0; frame < frameCount; ++frame) {
        solver.step(mesh, dt);
    }

    std::ifstream baseline(baselinePath());
    REQUIRE(baseline.is_open());

    std::size_t expectedCount = 0;
    baseline >> expectedCount;
    REQUIRE(mesh.particles().size() == expectedCount);

    const float tolerance = 1e-4f;
    for (std::size_t i = 0; i < expectedCount; ++i) {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        baseline >> x >> y >> z;

        const glm::vec3& actual = mesh.particles()[i].position;
        INFO("particle " << i);
        REQUIRE(actual.x == Approx(x).margin(tolerance));
        REQUIRE(actual.y == Approx(y).margin(tolerance));
        REQUIRE(actual.z == Approx(z).margin(tolerance));
    }
}
