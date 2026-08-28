#pragma once

#include <glm/vec3.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/CollisionWorld.hpp"

namespace clothsim {

// Declared outside Solver (rather than nested) to sidestep a GCC quirk where
// a nested aggregate type with default member initializers fails to resolve
// as a default argument value of its own enclosing class's constructor.
struct SolverParams {
    glm::vec3 gravity{0.0f, -9.81f, 0.0f};
    float damping = 0.99f; // velocity retained per step (1.0 = none)

    // Number of constraint-relaxation passes per step (Jakobsen-style). More
    // iterations converge closer to perfectly inextensible cloth at the cost
    // of CPU time; 3-5 is a common real-time-friendly range.
    int constraintIterations = 5;
};

// Verlet integration plus iterative position-based constraint relaxation.
// Unlike Step 3's force-based springs, this never overshoots: each spring is
// satisfied by directly moving its two particles to the correct rest
// distance (weighted by inverse mass, respecting pinned particles), so the
// solver stays stable regardless of how stiff the cloth "feels".
class Solver {
public:
    explicit Solver(SolverParams params = SolverParams());

    // Static colliders (planes, spheres) to test cloth particles against
    // each step. Pass nullptr (the default) to run with no collision world.
    void setCollisionWorld(const CollisionWorld* collisionWorld) { m_collisionWorld = collisionWorld; }

    // Advances the simulation by one step of size dt (seconds).
    void step(ClothMesh& mesh, float dt);

private:
    SolverParams m_params;
    const CollisionWorld* m_collisionWorld = nullptr;

    void integrate(ClothMesh& mesh, float dt);
    void relaxConstraints(ClothMesh& mesh);
};

} // namespace clothsim
