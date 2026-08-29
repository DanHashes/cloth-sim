#include "clothsim/Solver.hpp"

#include <vector>

#include <glm/geometric.hpp>

namespace clothsim {

Solver::Solver(SolverParams params) : m_params(params) {}

void Solver::step(ClothMesh& mesh, float dt) {
    integrate(mesh, dt);
    relaxConstraints(mesh);

    // Self-collision runs before static collision so that, if pushing two
    // overlapping particles apart happens to shove one into a collider, the
    // static collision pass (the last word each frame) still catches it.
    if (m_params.selfCollisionDistance > 0.0f && m_collisionWorld != nullptr) {
        m_collisionWorld->resolveSelfCollisions(mesh, m_params.selfCollisionDistance);
    }

    if (m_collisionWorld != nullptr) {
        m_collisionWorld->resolveCollisions(mesh);
    }
}

void Solver::integrate(ClothMesh& mesh, float dt) {
    // Gravity is the only force here now -- springs are no longer forces at
    // all, they're hard constraints resolved in relaxConstraints() below.
    for (Particle& p : mesh.particles()) {
        if (p.pinned) {
            p.previousPosition = p.position;
            continue;
        }

        const glm::vec3 velocityTerm = (p.position - p.previousPosition) * m_params.damping;
        const glm::vec3 acceleration = m_params.gravity + m_params.wind;
        const glm::vec3 newPosition = p.position + velocityTerm + acceleration * dt * dt;

        p.previousPosition = p.position;
        p.position = newPosition;
    }
}

void Solver::relaxConstraints(ClothMesh& mesh) {
    std::vector<Particle>& particles = mesh.particles();

    // Jakobsen-style relaxation: repeatedly walk every spring and directly
    // pull/push its two particles apart or together until they sit exactly
    // restLength apart. Each pass only partially resolves every spring
    // (satisfying one spring can un-satisfy a neighboring one), so we repeat
    // several passes per frame until the whole mesh converges toward a
    // consistent shape. This never overshoots the way a force can, because
    // we're setting position directly rather than integrating an
    // acceleration -- that's what makes it unconditionally stable.
    for (int iteration = 0; iteration < m_params.constraintIterations; ++iteration) {
        for (const SpringConstraint& spring : mesh.springs()) {
            Particle& a = particles[spring.particleA];
            Particle& b = particles[spring.particleB];

            const float invMassSum = a.invMass + b.invMass;
            if (invMassSum <= 0.0f) {
                continue; // both endpoints pinned, nothing to correct
            }

            const glm::vec3 delta = b.position - a.position;
            const float dist = glm::length(delta);
            if (dist < 1e-8f) {
                continue;
            }

            const float correctionMagnitude = (dist - spring.restLength) / dist;
            const glm::vec3 correction = delta * correctionMagnitude;

            a.position += correction * (a.invMass / invMassSum);
            b.position -= correction * (b.invMass / invMassSum);
        }
    }
}

} // namespace clothsim
