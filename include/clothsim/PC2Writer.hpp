#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

namespace clothsim {

// Writes Blender's PC2 ("Point Cache 2") binary vertex-cache format: a fixed
// 32-byte header followed by raw float32 xyz positions, one vertex after
// another, one frame ("sample") after another. This is the format Blender's
// Mesh Cache modifier reads natively -- no custom import code needed, as
// long as the mesh the modifier is applied to has exactly vertexCount
// vertices in the same order used here.
//
// Header layout (32 bytes total), per the PC2 spec:
//   char  cacheSignature[12]  "POINTCACHE2\0"
//   int32 fileVersion         1
//   int32 numPoints           vertices per sample
//   float startFrame
//   float sampleRate          frames per sample
//   int32 numSamples          total frame count
class PC2Writer {
public:
    // Opens `path` for writing and immediately writes a header with a
    // placeholder frame count (the true count isn't known until frames have
    // actually been written). Throws std::runtime_error if the file can't
    // be opened.
    explicit PC2Writer(const std::string& path, int vertexCount, float startFrame = 0.0f, float sampleRate = 1.0f);

    ~PC2Writer();

    PC2Writer(const PC2Writer&) = delete;
    PC2Writer& operator=(const PC2Writer&) = delete;

    // Appends one frame of vertex positions; positions.size() must equal
    // the vertexCount passed to the constructor.
    void writeFrame(const std::vector<glm::vec3>& positions);

    // Patches the header's frame count with the true number of frames
    // written and closes the file. Safe to call more than once; the
    // destructor calls it automatically if not already finalized.
    void finalize();

    int frameCount() const { return m_frameCount; }

private:
    std::ofstream m_file;
    int m_vertexCount;
    int m_frameCount = 0;
    std::streampos m_frameCountFieldPos{};
};

} // namespace clothsim
