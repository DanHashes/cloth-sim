#include <iostream>

#include "clothsim/ClothMesh.hpp"

int main() {
    std::cout << "cloth-sim init" << std::endl;

    // Temporary smoke test: build a small grid and print its topology to
    // confirm particles/springs/triangles are generated correctly. No
    // physics yet -- that starts in Step 3.
    const clothsim::ClothMesh cloth(/*width=*/2.0f, /*height=*/2.0f, /*resX=*/10, /*resY=*/10);

    std::cout << "particles: " << cloth.particles().size() << std::endl;
    std::cout << "springs:   " << cloth.springs().size() << std::endl;
    std::cout << "triangles: " << cloth.indices().size() / 3 << std::endl;
    std::cout << "normals:   " << cloth.normals().size() << std::endl;

    return 0;
}
