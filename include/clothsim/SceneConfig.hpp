#pragma once

#include <string>

#include <glm/vec3.hpp>

namespace clothsim {

enum class PinMode { None, TopRow, LeftColumn };

// Every setting needed to build and simulate one scene, whether it came
// from a scene file or command-line flags (main.cpp's CLI parser fills the
// exact same struct so both paths behave identically).
struct SceneConfig {
    float width = 2.0f;
    float height = 2.0f;
    int resX = 20;
    int resY = 20;

    int constraintIterations = 5;
    float damping = 0.98f;
    float gravity = 9.81f;
    glm::vec3 wind{0.0f, 0.0f, 0.0f};
    float selfCollisionDistance = 0.0f;
    PinMode pinMode = PinMode::TopRow;

    // Applied to the freshly-built flat grid before pinning: center
    // recenters it in X/Z (useful for dropping cloth onto something at the
    // origin), offset then translates it by an arbitrary amount.
    bool center = false;
    glm::vec3 offset{0.0f, 0.0f, 0.0f};

    bool hasSphere = false;
    glm::vec3 sphereCenter{0.0f};
    float sphereRadius = 0.5f;

    bool hasPlane = false;
    glm::vec3 planePoint{0.0f};
    glm::vec3 planeNormal{0.0f, 1.0f, 0.0f};

    int frameCount = 300;
    float fps = 60.0f;
    std::string outputPath = "cloth_output.pc2";
};

// Parses a simple whitespace-separated "key value..." text scene file: one
// setting per line, '#' starts a comment (to end of line), blank lines are
// ignored. See scenes/*.txt for examples and the full list of keys.
// Throws std::runtime_error if the file can't be opened or contains an
// unrecognized key / malformed value.
SceneConfig loadSceneConfig(const std::string& path);

} // namespace clothsim
