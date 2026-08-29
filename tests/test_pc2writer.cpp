#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/vec3.hpp>

#include "clothsim/PC2Writer.hpp"

using Catch::Approx;

TEST_CASE("PC2Writer produces a byte-exact header and vertex stream", "[pc2writer]") {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "clothsim_test_pc2writer.pc2";

    {
        clothsim::PC2Writer writer(path.string(), /*vertexCount=*/2, /*startFrame=*/0.0f, /*sampleRate=*/24.0f);
        writer.writeFrame({glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(-1.0f, 0.5f, 0.25f)});
        writer.writeFrame({glm::vec3(4.0f, 5.0f, 6.0f), glm::vec3(7.0f, 8.0f, 9.0f)});
        // Destructor runs here, finalizing the header and closing the file.
    }

    std::ifstream file(path, std::ios::binary);
    REQUIRE(file.is_open());

    char signature[12] = {};
    file.read(signature, sizeof(signature));
    REQUIRE(std::string(signature, 11) == "POINTCACHE2");
    REQUIRE(signature[11] == '\0');

    std::int32_t version = 0;
    std::int32_t numPoints = 0;
    float startFrame = -1.0f;
    float sampleRate = -1.0f;
    std::int32_t numSamples = 0;
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    file.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));
    file.read(reinterpret_cast<char*>(&startFrame), sizeof(startFrame));
    file.read(reinterpret_cast<char*>(&sampleRate), sizeof(sampleRate));
    file.read(reinterpret_cast<char*>(&numSamples), sizeof(numSamples));

    REQUIRE(version == 1);
    REQUIRE(numPoints == 2);
    REQUIRE(startFrame == Approx(0.0f));
    REQUIRE(sampleRate == Approx(24.0f));
    REQUIRE(numSamples == 2); // patched by finalize() from the true frame count written

    auto readFloat = [&file]() {
        float v = 0.0f;
        file.read(reinterpret_cast<char*>(&v), sizeof(v));
        return v;
    };

    // Frame 0, vertex 0.
    REQUIRE(readFloat() == Approx(1.0f));
    REQUIRE(readFloat() == Approx(2.0f));
    REQUIRE(readFloat() == Approx(3.0f));
    // Frame 0, vertex 1.
    REQUIRE(readFloat() == Approx(-1.0f));
    REQUIRE(readFloat() == Approx(0.5f));
    REQUIRE(readFloat() == Approx(0.25f));
    // Frame 1, vertex 0.
    REQUIRE(readFloat() == Approx(4.0f));
    REQUIRE(readFloat() == Approx(5.0f));
    REQUIRE(readFloat() == Approx(6.0f));
    // Frame 1, vertex 1.
    REQUIRE(readFloat() == Approx(7.0f));
    REQUIRE(readFloat() == Approx(8.0f));
    REQUIRE(readFloat() == Approx(9.0f));

    const std::uintmax_t expectedSize = 32 + 2 * 2 * 12;
    REQUIRE(std::filesystem::file_size(path) == expectedSize);

    file.close(); // Windows locks open file handles; must close before remove()
    std::filesystem::remove(path);
}

TEST_CASE("PC2Writer rejects a frame with the wrong vertex count", "[pc2writer]") {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "clothsim_test_pc2writer_bad.pc2";

    clothsim::PC2Writer writer(path.string(), /*vertexCount=*/3, 0.0f, 30.0f);
    REQUIRE_THROWS_AS(writer.writeFrame({glm::vec3(0.0f), glm::vec3(0.0f)}), std::runtime_error);

    writer.finalize();
    std::filesystem::remove(path);
}
