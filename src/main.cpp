#include <algorithm>
#include <iostream>
#include <limits>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/CollisionWorld.hpp"
#include "clothsim/Solver.hpp"

namespace {

// Brute-force O(n^2) pairwise minimum distance -- deliberately not using
// the spatial hash grid here, so this serves as an independent check that
// self-collision is actually holding particles apart (rather than the
// grid's own bookkeeping just reporting itself as consistent).
float bruteForceMinPairwiseDistance(const clothsim::ClothMesh& cloth) {
    const std::vector<clothsim::Particle>& particles = cloth.particles();
    float minDist = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < particles.size(); ++i) {
        for (std::size_t j = i + 1; j < particles.size(); ++j) {
            minDist = std::min(minDist, glm::length(particles[i].position - particles[j].position));
        }
    }
    return minDist;
}

void printFrameStats(int frame, const clothsim::ClothMesh& cloth) {
    glm::vec3 bboxMin(std::numeric_limits<float>::max());
    glm::vec3 bboxMax(std::numeric_limits<float>::lowest());
    for (const clothsim::Particle& p : cloth.particles()) {
        bboxMin = glm::min(bboxMin, p.position);
        bboxMax = glm::max(bboxMax, p.position);
    }

    std::cout << "frame " << frame
              << " | bbox y [" << bboxMin.y << ", " << bboxMax.y << "]"
              << " | min pairwise distance " << bruteForceMinPairwiseDistance(cloth) << std::endl;
}

} // namespace

int main() {
    std::cout << "cloth-sim init" << std::endl;

    clothsim::ClothMesh cloth(2.0f, 2.0f, /*resX=*/20, /*resY=*/20);
    std::cout << "particles: " << cloth.particles().size() << std::endl;
    std::cout << "springs:   " << cloth.springs().size() << std::endl;

    // Pin the two top corners much closer together than the top edge's
    // natural (rest-length) span. The edge can't stretch to close the gap
    // -- constraints hold rest length -- so it has to buckle and fold
    // instead, dragging the rest of the sheet into a crumpled, self-
    // overlapping mess as it falls. This is the stress test self-collision
    // needs to survive.
    const std::size_t leftCorner = cloth.particleIndex(0, 0);
    const std::size_t rightCorner = cloth.particleIndex(cloth.resolutionX() - 1, 0);
    cloth.particles()[leftCorner].pinned = true;
    cloth.particles()[leftCorner].invMass = 0.0f;
    cloth.particles()[rightCorner].pinned = true;
    cloth.particles()[rightCorner].invMass = 0.0f;
    cloth.particles()[rightCorner].position = glm::vec3(0.3f, 0.0f, 0.0f);
    cloth.particles()[rightCorner].previousPosition = cloth.particles()[rightCorner].position;

    clothsim::CollisionWorld world;
    world.addPlane(clothsim::PlaneCollider{glm::vec3(0.0f, -2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)});

    clothsim::SolverParams params;
    params.constraintIterations = 6;
    params.damping = 0.96f;
    params.selfCollisionDistance = 0.05f; // particles may never get closer than this
    clothsim::Solver solver(params);
    solver.setCollisionWorld(&world);

    const float dt = 1.0f / 60.0f;
    const int frameCount = 300;

    for (int frame = 1; frame <= frameCount; ++frame) {
        solver.step(cloth, dt);
        if (frame % 20 == 0) {
            printFrameStats(frame, cloth);
        }
    }

    return 0;
}
