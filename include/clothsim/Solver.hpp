#pragma once

#include <glm/vec3.hpp>

#include "clothsim/ClothMesh.hpp"

namespace clothsim {

// Declared outside Solver (rather than nested) to sidestep a GCC quirk where
// a nested aggregate type with default member initializers fails to resolve
// as a default argument value of its own enclosing class's constructor.
struct SolverParams {
    glm::vec3 gravity{0.0f, -9.81f, 0.0f};
    float damping = 0.99f;          // velocity retained per step (1.0 = none)
    float springStiffness = 4000.0f; // Hooke's law spring constant, F = -k * (dist - restLength)
};

// Naive baseline solver: Verlet integration plus force-based (Hooke's law)
// springs. No constraint relaxation -- this is the deliberately unstable
// "before" state that Step 4 replaces with position-based constraints.
class Solver {
public:
    explicit Solver(SolverParams params = SolverParams());

    // Advances the simulation by one step of size dt (seconds).
    void step(ClothMesh& mesh, float dt);

private:
    SolverParams m_params;
};

} // namespace clothsim
