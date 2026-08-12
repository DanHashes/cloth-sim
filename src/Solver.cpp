#include "clothsim/Solver.hpp"

#include <vector>

#include <glm/geometric.hpp>

namespace clothsim {

Solver::Solver(SolverParams params) : m_params(params) {}

void Solver::step(ClothMesh& mesh, float dt) {
    std::vector<Particle>& particles = mesh.particles();
    std::vector<glm::vec3> forces(particles.size(), glm::vec3(0.0f));

    // Naive force-based springs: each spring pulls/pushes its two particles
    // directly toward their rest length, proportional to how far they've
    // stretched or compressed (Hooke's law). Nothing here prevents the
    // stretch from growing unbounded step to step -- that's the point of
    // this baseline.
    for (const SpringConstraint& spring : mesh.springs()) {
        const glm::vec3& posA = particles[spring.particleA].position;
        const glm::vec3& posB = particles[spring.particleB].position;

        const glm::vec3 delta = posB - posA;
        const float dist = glm::length(delta);
        if (dist < 1e-8f) {
            continue;
        }

        const glm::vec3 direction = delta / dist;
        const float stretch = dist - spring.restLength;
        const glm::vec3 forceOnA = direction * (m_params.springStiffness * stretch);

        forces[spring.particleA] += forceOnA;
        forces[spring.particleB] -= forceOnA;
    }

    // Verlet integration: velocity is implicit in (position - previousPosition),
    // so we never store it explicitly. damping < 1 bleeds off that implicit
    // velocity each step to emulate air resistance / energy loss.
    for (std::size_t i = 0; i < particles.size(); ++i) {
        Particle& p = particles[i];
        if (p.pinned) {
            p.previousPosition = p.position;
            continue;
        }

        const glm::vec3 acceleration = m_params.gravity + forces[i] * p.invMass;
        const glm::vec3 velocityTerm = (p.position - p.previousPosition) * m_params.damping;
        const glm::vec3 newPosition = p.position + velocityTerm + acceleration * dt * dt;

        p.previousPosition = p.position;
        p.position = newPosition;
    }
}

} // namespace clothsim
