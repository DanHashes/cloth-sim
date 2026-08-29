#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/geometric.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/Solver.hpp"

using Catch::Approx;

TEST_CASE("ClothMesh generates springs with correct hand-computed rest lengths", "[springs][topology]") {
    // 3x3 grid, 1 unit spacing in both directions -- easy numbers to verify
    // by hand: structural springs should be exactly 1.0 apart, diagonal
    // (shear) springs sqrt(2), and bend springs (skip-one neighbor) 2.0.
    clothsim::ClothMesh mesh(/*width=*/2.0f, /*height=*/2.0f, /*resX=*/3, /*resY=*/3);

    bool foundStructural = false;
    bool foundShear = false;
    bool foundBend = false;

    for (const clothsim::SpringConstraint& spring : mesh.springs()) {
        switch (spring.type) {
            case clothsim::SpringType::Structural:
                REQUIRE(spring.restLength == Approx(1.0f));
                foundStructural = true;
                break;
            case clothsim::SpringType::Shear:
                REQUIRE(spring.restLength == Approx(std::sqrt(2.0f)));
                foundShear = true;
                break;
            case clothsim::SpringType::Bend:
                REQUIRE(spring.restLength == Approx(2.0f));
                foundBend = true;
                break;
        }
    }

    REQUIRE(foundStructural);
    REQUIRE(foundShear);
    REQUIRE(foundBend);
}

TEST_CASE("ClothMesh spring counts match hand-computed grid combinatorics", "[springs][topology]") {
    // For an resX x resY grid: structural = (resX-1)*resY + resX*(resY-1),
    // shear = 2*(resX-1)*(resY-1), bend = (resX-2)*resY + resX*(resY-2)
    // (bend terms only counted when resX/resY >= 3, otherwise 0).
    clothsim::ClothMesh mesh(2.0f, 2.0f, /*resX=*/10, /*resY=*/10);

    const int resX = 10;
    const int resY = 10;
    const std::size_t expectedStructural = static_cast<std::size_t>((resX - 1) * resY + resX * (resY - 1));
    const std::size_t expectedShear = static_cast<std::size_t>(2 * (resX - 1) * (resY - 1));
    const std::size_t expectedBend = static_cast<std::size_t>((resX - 2) * resY + resX * (resY - 2));

    std::size_t structuralCount = 0;
    std::size_t shearCount = 0;
    std::size_t bendCount = 0;
    for (const clothsim::SpringConstraint& spring : mesh.springs()) {
        switch (spring.type) {
            case clothsim::SpringType::Structural: ++structuralCount; break;
            case clothsim::SpringType::Shear: ++shearCount; break;
            case clothsim::SpringType::Bend: ++bendCount; break;
        }
    }

    REQUIRE(structuralCount == expectedStructural);
    REQUIRE(shearCount == expectedShear);
    REQUIRE(bendCount == expectedBend);
    REQUIRE(mesh.particles().size() == static_cast<std::size_t>(resX * resY));
}

TEST_CASE("Constraint relaxation moves both particles to satisfy rest length, weighted by inverse mass", "[springs][solver]") {
    // Isolate pure constraint math from gravity/integration: zero gravity
    // means integrate() is a no-op for particles starting at rest, so a
    // single relaxation pass over exactly one spring is fully hand-checkable.
    clothsim::ClothMesh mesh(1.0f, 1.0f, 2, 2);
    mesh.springs().clear();
    mesh.springs().push_back(clothsim::SpringConstraint{
        /*particleA=*/0, /*particleB=*/1, /*restLength=*/0.6f, clothsim::SpringType::Structural});

    mesh.particles()[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
    mesh.particles()[0].previousPosition = mesh.particles()[0].position;
    mesh.particles()[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
    mesh.particles()[1].previousPosition = mesh.particles()[1].position;

    clothsim::SolverParams params;
    params.gravity = glm::vec3(0.0f);
    params.constraintIterations = 1;
    clothsim::Solver solver(params);

    solver.step(mesh, 1.0f / 60.0f);

    // Hand computation: dist=1.0, restLength=0.6, invMassSum=2,
    // correctionMagnitude=(1.0-0.6)/1.0=0.4, correction=(0.4,0,0).
    // a += correction*(1/2) = (0.2,0,0); b -= correction*(1/2) -> (0.8,0,0).
    REQUIRE(mesh.particles()[0].position.x == Approx(0.2f));
    REQUIRE(mesh.particles()[1].position.x == Approx(0.8f));
    REQUIRE(glm::length(mesh.particles()[1].position - mesh.particles()[0].position) == Approx(0.6f));
}

TEST_CASE("A pinned particle absorbs none of the constraint correction", "[springs][solver]") {
    clothsim::ClothMesh mesh(1.0f, 1.0f, 2, 2);
    mesh.springs().clear();
    mesh.springs().push_back(clothsim::SpringConstraint{0, 1, 0.6f, clothsim::SpringType::Structural});

    mesh.particles()[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
    mesh.particles()[0].previousPosition = mesh.particles()[0].position;
    mesh.particles()[0].pinned = true;
    mesh.particles()[0].invMass = 0.0f;

    mesh.particles()[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
    mesh.particles()[1].previousPosition = mesh.particles()[1].position;

    clothsim::SolverParams params;
    params.gravity = glm::vec3(0.0f);
    params.constraintIterations = 1;
    clothsim::Solver solver(params);

    solver.step(mesh, 1.0f / 60.0f);

    // Hand computation: invMassSum=1 (only particle B counts), correction
    // magnitude 0.4; A gets 0/1 of it (stays put), B gets all of it.
    REQUIRE(mesh.particles()[0].position.x == Approx(0.0f));
    REQUIRE(mesh.particles()[1].position.x == Approx(0.6f));
}
