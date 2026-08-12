#include "clothsim/ClothMesh.hpp"

#include <cassert>

#include <glm/geometric.hpp>

namespace clothsim {

ClothMesh::ClothMesh(float width, float height, int resX, int resY)
    : m_resX(resX), m_resY(resY), m_width(width), m_height(height) {
    assert(resX >= 2 && resY >= 2 && "ClothMesh requires at least a 2x2 particle grid");

    buildParticles();
    buildSprings();
    buildTriangles();
    computeNormals();
}

std::size_t ClothMesh::particleIndex(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(m_resX) + static_cast<std::size_t>(x);
}

void ClothMesh::buildParticles() {
    m_particles.resize(static_cast<std::size_t>(m_resX) * static_cast<std::size_t>(m_resY));

    const float dx = m_width / static_cast<float>(m_resX - 1);
    const float dy = m_height / static_cast<float>(m_resY - 1);

    // Lay the grid flat in the XZ plane (y = 0), top row at z = 0. A pinned
    // top row plus gravity in -Y is what makes it drape/hang like cloth.
    for (int y = 0; y < m_resY; ++y) {
        for (int x = 0; x < m_resX; ++x) {
            Particle& p = m_particles[particleIndex(x, y)];
            p.position = glm::vec3(static_cast<float>(x) * dx, 0.0f, static_cast<float>(y) * dy);
            p.previousPosition = p.position;
            p.invMass = 1.0f;
            p.pinned = false;
        }
    }
}

void ClothMesh::buildSprings() {
    m_springs.clear();

    auto addSpring = [this](int ax, int ay, int bx, int by, SpringType type) {
        const std::size_t a = particleIndex(ax, ay);
        const std::size_t b = particleIndex(bx, by);
        const float restLength = glm::length(m_particles[a].position - m_particles[b].position);
        m_springs.push_back(SpringConstraint{a, b, restLength, type});
    };

    for (int y = 0; y < m_resY; ++y) {
        for (int x = 0; x < m_resX; ++x) {
            // Structural: right and down neighbors.
            if (x + 1 < m_resX) {
                addSpring(x, y, x + 1, y, SpringType::Structural);
            }
            if (y + 1 < m_resY) {
                addSpring(x, y, x, y + 1, SpringType::Structural);
            }

            // Shear: both diagonals of the quad to the lower-right.
            if (x + 1 < m_resX && y + 1 < m_resY) {
                addSpring(x, y, x + 1, y + 1, SpringType::Shear);
                addSpring(x + 1, y, x, y + 1, SpringType::Shear);
            }

            // Bend: skip-one neighbors, resists sharp folding.
            if (x + 2 < m_resX) {
                addSpring(x, y, x + 2, y, SpringType::Bend);
            }
            if (y + 2 < m_resY) {
                addSpring(x, y, x, y + 2, SpringType::Bend);
            }
        }
    }
}

void ClothMesh::buildTriangles() {
    m_indices.clear();
    m_indices.reserve(static_cast<std::size_t>(m_resX - 1) * static_cast<std::size_t>(m_resY - 1) * 6);

    for (int y = 0; y + 1 < m_resY; ++y) {
        for (int x = 0; x + 1 < m_resX; ++x) {
            const unsigned int topLeft = static_cast<unsigned int>(particleIndex(x, y));
            const unsigned int topRight = static_cast<unsigned int>(particleIndex(x + 1, y));
            const unsigned int bottomLeft = static_cast<unsigned int>(particleIndex(x, y + 1));
            const unsigned int bottomRight = static_cast<unsigned int>(particleIndex(x + 1, y + 1));

            m_indices.push_back(topLeft);
            m_indices.push_back(bottomLeft);
            m_indices.push_back(topRight);

            m_indices.push_back(topRight);
            m_indices.push_back(bottomLeft);
            m_indices.push_back(bottomRight);
        }
    }
}

void ClothMesh::computeNormals() {
    m_normals.assign(m_particles.size(), glm::vec3(0.0f));

    for (std::size_t i = 0; i + 2 < m_indices.size(); i += 3) {
        const unsigned int ia = m_indices[i];
        const unsigned int ib = m_indices[i + 1];
        const unsigned int ic = m_indices[i + 2];

        const glm::vec3& a = m_particles[ia].position;
        const glm::vec3& b = m_particles[ib].position;
        const glm::vec3& c = m_particles[ic].position;

        const glm::vec3 faceNormal = glm::cross(b - a, c - a);

        m_normals[ia] += faceNormal;
        m_normals[ib] += faceNormal;
        m_normals[ic] += faceNormal;
    }

    for (glm::vec3& n : m_normals) {
        const float len = glm::length(n);
        if (len > 1e-8f) {
            n /= len;
        }
    }
}

} // namespace clothsim
