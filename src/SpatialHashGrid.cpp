#include "clothsim/SpatialHashGrid.hpp"

#include <cmath>

namespace clothsim {

SpatialHashGrid::SpatialHashGrid(float cellSize) : m_cellSize(cellSize) {}

void SpatialHashGrid::build(const std::vector<Particle>& particles) {
    m_cells.clear();
    m_cells.reserve(particles.size());

    for (std::size_t i = 0; i < particles.size(); ++i) {
        const CellKey key = cellKeyFor(particles[i].position);
        m_cells[key].push_back(i);
    }
}

CellKey SpatialHashGrid::cellKeyFor(const glm::vec3& position) const {
    // std::floor (not truncation) so negative coordinates bucket correctly:
    // -0.1 and 0.1 must land in different cells when cellSize is 1.
    return CellKey{
        static_cast<int>(std::floor(position.x / m_cellSize)),
        static_cast<int>(std::floor(position.y / m_cellSize)),
        static_cast<int>(std::floor(position.z / m_cellSize))};
}

} // namespace clothsim
