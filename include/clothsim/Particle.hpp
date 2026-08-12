#pragma once

// Vector math uses GLM (https://github.com/g-truc/glm), fetched via CMake
// FetchContent. Header-only, industry-standard in OpenGL/Vulkan projects,
// so we get vec3 arithmetic, dot/cross/normalize, etc. without hand-rolling it.
#include <glm/vec3.hpp>

namespace clothsim {

struct Particle {
    glm::vec3 position{0.0f};
    glm::vec3 previousPosition{0.0f};
    float invMass = 1.0f;
    bool pinned = false;
};

} // namespace clothsim
