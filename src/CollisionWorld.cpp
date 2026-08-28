#include "clothsim/CollisionWorld.hpp"

#include <glm/geometric.hpp>

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

} // namespace clothsim
