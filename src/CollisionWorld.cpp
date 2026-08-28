#include "clothsim/CollisionWorld.hpp"

#include <glm/geometric.hpp>

#include "clothsim/SpatialHashGrid.hpp"

namespace clothsim {

void CollisionWorld::addPlane(const PlaneCollider& plane) {
    m_planes.push_back(plane);
}

void CollisionWorld::addSphere(const SphereCollider& sphere) {
    m_spheres.push_back(sphere);
}

void CollisionWorld::resolveCollisions(ClothMesh& mesh) const {
    for (Particle& p : mesh.particles()) {
        if (p.pinned) {
            continue;
        }
        for (const PlaneCollider& plane : m_planes) {
            resolvePlane(p, plane);
        }
        for (const SphereCollider& sphere : m_spheres) {
            resolveSphere(p, sphere);
        }
    }
}

void CollisionWorld::resolvePlane(Particle& p, const PlaneCollider& plane) const {
    const float distance = glm::dot(p.position - plane.point, plane.normal);
    if (distance >= 0.0f) {
        return; // on the allowed side, nothing to do
    }

    // Push the particle back onto the plane surface and kill the velocity
    // component that drove it through -- otherwise Verlet would reconstruct
    // that same into-the-surface velocity next step from (position -
    // previousPosition) and the particle would tunnel through again.
    p.position -= plane.normal * distance;
    p.previousPosition = p.position;
}

void CollisionWorld::resolveSphere(Particle& p, const SphereCollider& sphere) const {
    const glm::vec3 offset = p.position - sphere.center;
    const float dist = glm::length(offset);
    if (dist >= sphere.radius || dist < 1e-8f) {
        return; // outside the sphere, or exactly at its center (degenerate)
    }

    const glm::vec3 normal = offset / dist;
    p.position = sphere.center + normal * sphere.radius;
    p.previousPosition = p.position;
}

void CollisionWorld::resolveSelfCollisions(ClothMesh& mesh, float minSeparation) const {
    std::vector<Particle>& particles = mesh.particles();

    // Cell size == minSeparation: any two particles closer than
    // minSeparation are guaranteed to fall in the same cell or one directly
    // adjacent to it, so the 27-cell neighborhood search never misses a
    // true collision candidate.
    SpatialHashGrid grid(minSeparation);
    grid.build(particles);

    grid.forEachCandidatePair([&](std::size_t i, std::size_t j) {
        resolveParticlePair(particles[i], particles[j], minSeparation);
    });
}

void CollisionWorld::resolveParticlePair(Particle& a, Particle& b, float minSeparation) const {
    const float invMassSum = a.invMass + b.invMass;
    if (invMassSum <= 0.0f) {
        return; // both pinned, nothing to correct
    }

    const glm::vec3 delta = b.position - a.position;
    const float dist = glm::length(delta);
    if (dist >= minSeparation || dist < 1e-8f) {
        return; // not overlapping, or coincident (degenerate, ignore)
    }

    const glm::vec3 direction = delta / dist;
    const float overlap = minSeparation - dist;

    // Push a away from b and b away from a, split by inverse mass so a
    // pinned particle (invMass 0) never moves and its partner absorbs the
    // whole correction.
    a.position -= direction * (overlap * (a.invMass / invMassSum));
    b.position += direction * (overlap * (b.invMass / invMassSum));
}

} // namespace clothsim
