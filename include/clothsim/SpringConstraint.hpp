#pragma once

#include <cstddef>

namespace clothsim {

enum class SpringType {
    Structural, // grid-adjacent neighbors (horizontal/vertical)
    Shear,      // diagonal neighbors
    Bend        // skip-one neighbors, resists folding
};

struct SpringConstraint {
    std::size_t particleA;
    std::size_t particleB;
    float restLength;
    SpringType type;
};

} // namespace clothsim
