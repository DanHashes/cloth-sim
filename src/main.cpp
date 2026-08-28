#include <algorithm>
#include <iostream>
#include <limits>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/CollisionWorld.hpp"
#include "clothsim/Solver.hpp"

namespace {

// Reports the bounding box (to see the cloth falling/settling) and how deep
// the worst-offending particle is inside the sphere, if at all. A negative
// value here would mean collision resolution failed and cloth is tunneling
// through the sphere; it should stay >= 0 every frame once particles have
// been pushed to the surface.
void printFrameStats(int frame, const clothsim::ClothMesh& cloth, const glm::vec3& sphereCenter, float sphereRadius) {
    glm::vec3 bboxMin(std::numeric_limits<float>::max());
    glm::vec3 bboxMax(std::numeric_limits<float>::lowest());
    float minSurfaceDistance = std::numeric_limits<float>::max();

    for (const clothsim::Particle& p : cloth.particles()) {
        bboxMin = glm::min(bboxMin, p.position);
        bboxMax = glm::max(bboxMax, p.position);
        const float surfaceDistance = glm::length(p.position - sphereCenter) - sphereRadius;
        minSurfaceDistance = std::min(minSurfaceDistance, surfaceDistance);
    }

    std::cout << "frame " << frame
              << " | bbox y [" << bboxMin.y << ", " << bboxMax.y << "]"
              << " | min distance to sphere surface " << minSurfaceDistance << std::endl;
}

} // namespace

int main() {
    std::cout << "cloth-sim init" << std::endl;

    const float clothWidth = 2.0f;
    const float clothHeight = 2.0f;
    clothsim::ClothMesh cloth(clothWidth, clothHeight, /*resX=*/20, /*resY=*/20);
    std::cout << "particles: " << cloth.particles().size() << std::endl;
    std::cout << "springs:   " << cloth.springs().size() << std::endl;

    // Center the cloth over the sphere and lift it above, unpinned, so it
    // falls freely like a tablecloth being dropped rather than a curtain
    // hanging from a fixed edge.
    for (clothsim::Particle& p : cloth.particles()) {
        p.position.x -= clothWidth * 0.5f;
        p.position.z -= clothHeight * 0.5f;
        p.position.y += 1.2f;
        p.previousPosition = p.position;
    }

    const glm::vec3 sphereCenter(0.0f, 0.0f, 0.0f);
    const float sphereRadius = 0.6f;

    clothsim::CollisionWorld world;
    world.addSphere(clothsim::SphereCollider{sphereCenter, sphereRadius});
    // Ground plane well below the sphere as a safety net for anything that
    // slides off its sides.
    world.addPlane(clothsim::PlaneCollider{glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)});

    clothsim::SolverParams params;
    params.constraintIterations = 5;
    params.damping = 0.96f;
    clothsim::Solver solver(params);
    solver.setCollisionWorld(&world);

    const float dt = 1.0f / 60.0f;
    const int frameCount = 200;

    for (int frame = 1; frame <= frameCount; ++frame) {
        solver.step(cloth, dt);
        if (frame % 10 == 0) {
            printFrameStats(frame, cloth, sphereCenter, sphereRadius);
        }
    }

    return 0;
}
