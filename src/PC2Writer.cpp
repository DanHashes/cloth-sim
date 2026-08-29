#include "clothsim/PC2Writer.hpp"

#include <cstdint>
#include <stdexcept>

namespace clothsim {

namespace {
constexpr char kSignature[12] = {'P', 'O', 'I', 'N', 'T', 'C', 'A', 'C', 'H', 'E', '2', '\0'};
constexpr std::int32_t kFileVersion = 1;

static_assert(sizeof(float) == 4, "PC2 format requires IEEE-754 32-bit floats");
} // namespace

PC2Writer::PC2Writer(const std::string& path, int vertexCount, float startFrame, float sampleRate)
    : m_file(path, std::ios::binary | std::ios::out), m_vertexCount(vertexCount) {
    if (!m_file.is_open()) {
        throw std::runtime_error("PC2Writer: failed to open file for writing: " + path);
    }

    m_file.write(kSignature, sizeof(kSignature));
    m_file.write(reinterpret_cast<const char*>(&kFileVersion), sizeof(kFileVersion));

    const std::int32_t numPoints = static_cast<std::int32_t>(vertexCount);
    m_file.write(reinterpret_cast<const char*>(&numPoints), sizeof(numPoints));
    m_file.write(reinterpret_cast<const char*>(&startFrame), sizeof(startFrame));
    m_file.write(reinterpret_cast<const char*>(&sampleRate), sizeof(sampleRate));

    // numSamples isn't known yet -- we don't require the caller to say
    // upfront how many frames they'll write. Remember this offset so
    // finalize() can come back and patch in the real count.
    m_frameCountFieldPos = m_file.tellp();
    const std::int32_t placeholderFrameCount = 0;
    m_file.write(reinterpret_cast<const char*>(&placeholderFrameCount), sizeof(placeholderFrameCount));
}

void PC2Writer::writeFrame(const std::vector<glm::vec3>& positions) {
    if (static_cast<int>(positions.size()) != m_vertexCount) {
        throw std::runtime_error("PC2Writer::writeFrame: position count does not match vertexCount");
    }

    for (const glm::vec3& p : positions) {
        const float xyz[3] = {p.x, p.y, p.z};
        m_file.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
    }
    ++m_frameCount;
}

void PC2Writer::finalize() {
    if (!m_file.is_open()) {
        return;
    }

    const std::int32_t finalCount = static_cast<std::int32_t>(m_frameCount);
    m_file.seekp(m_frameCountFieldPos);
    m_file.write(reinterpret_cast<const char*>(&finalCount), sizeof(finalCount));
    m_file.flush();
    m_file.close();
}

PC2Writer::~PC2Writer() {
    finalize();
}

} // namespace clothsim
