#pragma once

#include <cstddef>
#include <vector>

#include <glm/vec3.hpp>

#include "clothsim/Particle.hpp"
#include "clothsim/SpringConstraint.hpp"

namespace clothsim {

// Owns cloth topology and state: the particle grid, the springs connecting
// them, and the triangle mesh used for rendering/normals. No integration or
// collision logic lives here (see Solver, CollisionWorld) — this class has
// exactly one job: represent "what a piece of cloth is made of."
class ClothMesh {
public:
    // width/height: physical size of the cloth in world units.
    // resX/resY: number of particles across/down (each >= 2).
    ClothMesh(float width, float height, int resX, int resY);

    int resolutionX() const { return m_resX; }
    int resolutionY() const { return m_resY; }

    std::vector<Particle>& particles() { return m_particles; }
    const std::vector<Particle>& particles() const { return m_particles; }

    std::vector<SpringConstraint>& springs() { return m_springs; }
    const std::vector<SpringConstraint>& springs() const { return m_springs; }

    const std::vector<unsigned int>& indices() const { return m_indices; }

    const std::vector<glm::vec3>& normals() const { return m_normals; }

    // Recomputes per-vertex normals from the current particle positions.
    void computeNormals();

    // Maps a (column, row) grid coordinate to a flat particle index.
    std::size_t particleIndex(int x, int y) const;

private:
    int m_resX;
    int m_resY;
    float m_width;
    float m_height;

    std::vector<Particle> m_particles;
    std::vector<SpringConstraint> m_springs;
    std::vector<unsigned int> m_indices;
    std::vector<glm::vec3> m_normals;

    void buildParticles();
    void buildSprings();
    void buildTriangles();
};

} // namespace clothsim
