#include <iostream>
#include <limits>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/Solver.hpp"

namespace {

void printBoundingBoxAndMaxSpeed(int frame, const clothsim::ClothMesh& cloth) {
    glm::vec3 bboxMin(std::numeric_limits<float>::max());
    glm::vec3 bboxMax(std::numeric_limits<float>::lowest());
    float maxSpeed = 0.0f;

    for (const clothsim::Particle& p : cloth.particles()) {
        bboxMin = glm::min(bboxMin, p.position);
        bboxMax = glm::max(bboxMax, p.position);
        maxSpeed = std::max(maxSpeed, glm::length(p.position - p.previousPosition));
    }

    std::cout << "frame " << frame
              << " | bbox min (" << bboxMin.x << ", " << bboxMin.y << ", " << bboxMin.z << ")"
              << " max (" << bboxMax.x << ", " << bboxMax.y << ", " << bboxMax.z << ")"
              << " | max per-step displacement " << maxSpeed << std::endl;
}

} // namespace

int main() {
    std::cout << "cloth-sim init" << std::endl;

    clothsim::ClothMesh cloth(/*width=*/2.0f, /*height=*/2.0f, /*resX=*/10, /*resY=*/10);
    std::cout << "particles: " << cloth.particles().size() << std::endl;
    std::cout << "springs:   " << cloth.springs().size() << std::endl;
    std::cout << "triangles: " << cloth.indices().size() / 3 << std::endl;

    // Pin the top row (grid row 0) so the cloth hangs under gravity instead
    // of just free-falling as a rigid, unattached grid.
    for (int x = 0; x < cloth.resolutionX(); ++x) {
        clothsim::Particle& p = cloth.particles()[cloth.particleIndex(x, 0)];
        p.pinned = true;
        p.invMass = 0.0f;
    }

    // Naive baseline solver (Step 3): Verlet integration + force-based
    // Hooke's-law springs, no constraint relaxation. At this stiffness the
    // per-step spring correction overshoots the rest length by more than it
    // started with, so the error compounds every frame and the simulation
    // diverges to NaN within ~30 frames -- that explosion is deliberate,
    // and gets fixed by Step 4's constraint relaxation.
    clothsim::Solver solver;
    const float dt = 1.0f / 60.0f;
    const int frameCount = 120;

    for (int frame = 1; frame <= frameCount; ++frame) {
        solver.step(cloth, dt);
        if (frame % 10 == 0) {
            printBoundingBoxAndMaxSpeed(frame, cloth);
        }
    }

    return 0;
}
