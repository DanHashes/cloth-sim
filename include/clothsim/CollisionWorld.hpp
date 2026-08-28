#pragma once

#include <vector>

#include <glm/vec3.hpp>

#include "clothsim/ClothMesh.hpp"

namespace clothsim {

// Infinite plane defined by a point on the plane and its outward-facing
// normal (must be unit length). "Outward" is whichever side particles are
// allowed to occupy.
struct PlaneCollider {
    glm::vec3 point;
    glm::vec3 normal;
};

struct SphereCollider {
    glm::vec3 center;
    float radius;
};

// Holds a scene's static colliders and pushes any cloth particle that has
// penetrated one back out to its surface. Purely geometric -- it knows
// nothing about springs or integration, it only ever touches particle
// position (and previousPosition, to kill velocity into the surface so
// particles don't keep re-penetrating and jittering frame to frame).
class CollisionWorld {
public:
    void addPlane(const PlaneCollider& plane);
    void addSphere(const SphereCollider& sphere);

    // Projects every penetrating particle in the mesh back onto the nearest
    // collider surface it violated.
    void resolveCollisions(ClothMesh& mesh) const;

private:
    std::vector<PlaneCollider> m_planes;
    std::vector<SphereCollider> m_spheres;

    void resolvePlane(Particle& p, const PlaneCollider& plane) const;
    void resolveSphere(Particle& p, const SphereCollider& sphere) const;
};

} // namespace clothsim
