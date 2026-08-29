#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "clothsim/ClothMesh.hpp"
#include "clothsim/CollisionWorld.hpp"
#include "clothsim/PC2Writer.hpp"
#include "clothsim/SceneConfig.hpp"
#include "clothsim/Solver.hpp"

namespace {

void printUsage() {
    std::cout <<
        "cloth-sim: cloth physics simulator with PC2 point-cache export\n\n"
        "Usage: clothsim [options]\n"
        "  --scene PATH              load settings from a scene file (see scenes/*.txt);\n"
        "                            any flags after this override individual settings\n"
        "  --width F                 cloth width in world units (default 2.0)\n"
        "  --height F                cloth height in world units (default 2.0)\n"
        "  --resx N                  particles across (default 20)\n"
        "  --resy N                  particles down (default 20)\n"
        "  --iterations N            constraint relaxation passes per step (default 5)\n"
        "  --damping F               velocity retained per step, 0-1 (default 0.98)\n"
        "  --gravity F               gravity magnitude along -Y (default 9.81)\n"
        "  --wind X Y Z              constant wind acceleration (default 0 0 0)\n"
        "  --self-collision F        minimum particle separation, 0 disables (default 0)\n"
        "  --pin-mode MODE           none | top_row | left_column (default top_row)\n"
        "  --center                  recenter the flat grid in X/Z before pinning\n"
        "  --offset X Y Z            translate the flat grid before pinning (default 0 0 0)\n"
        "  --sphere CX CY CZ R       add a sphere collider\n"
        "  --plane PX PY PZ NX NY NZ add a plane collider\n"
        "  --frames N                number of frames to simulate (default 300)\n"
        "  --fps F                   simulation frame rate (default 60)\n"
        "  --output PATH             output .pc2 path (default cloth_output.pc2)\n"
        "  --help                    show this message\n";
}

clothsim::PinMode parsePinMode(const std::string& value) {
    if (value == "none") return clothsim::PinMode::None;
    if (value == "top_row") return clothsim::PinMode::TopRow;
    if (value == "left_column") return clothsim::PinMode::LeftColumn;
    throw std::runtime_error("--pin-mode must be one of: none, top_row, left_column (got '" + value + "')");
}

enum class ParseResult { Success, ShowHelp, Error };

ParseResult parseArgs(int argc, char** argv, clothsim::SceneConfig& config) {
    auto needArg = [&](int i) {
        if (i + 1 >= argc) {
            throw std::runtime_error("missing value for argument: " + std::string(argv[i]));
        }
    };

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];

            if (arg == "--help") {
                return ParseResult::ShowHelp;
            } else if (arg == "--scene") {
                needArg(i);
                config = clothsim::loadSceneConfig(argv[++i]);
            } else if (arg == "--width") {
                needArg(i);
                config.width = std::stof(argv[++i]);
            } else if (arg == "--height") {
                needArg(i);
                config.height = std::stof(argv[++i]);
            } else if (arg == "--resx") {
                needArg(i);
                config.resX = std::stoi(argv[++i]);
            } else if (arg == "--resy") {
                needArg(i);
                config.resY = std::stoi(argv[++i]);
            } else if (arg == "--iterations") {
                needArg(i);
                config.constraintIterations = std::stoi(argv[++i]);
            } else if (arg == "--damping") {
                needArg(i);
                config.damping = std::stof(argv[++i]);
            } else if (arg == "--gravity") {
                needArg(i);
                config.gravity = std::stof(argv[++i]);
            } else if (arg == "--wind") {
                if (i + 3 >= argc) {
                    throw std::runtime_error("--wind requires 3 values: X Y Z");
                }
                config.wind.x = std::stof(argv[++i]);
                config.wind.y = std::stof(argv[++i]);
                config.wind.z = std::stof(argv[++i]);
            } else if (arg == "--self-collision") {
                needArg(i);
                config.selfCollisionDistance = std::stof(argv[++i]);
            } else if (arg == "--pin-mode") {
                needArg(i);
                config.pinMode = parsePinMode(argv[++i]);
            } else if (arg == "--center") {
                config.center = true;
            } else if (arg == "--offset") {
                if (i + 3 >= argc) {
                    throw std::runtime_error("--offset requires 3 values: X Y Z");
                }
                config.offset.x = std::stof(argv[++i]);
                config.offset.y = std::stof(argv[++i]);
                config.offset.z = std::stof(argv[++i]);
            } else if (arg == "--sphere") {
                if (i + 4 >= argc) {
                    throw std::runtime_error("--sphere requires 4 values: CX CY CZ R");
                }
                config.hasSphere = true;
                config.sphereCenter.x = std::stof(argv[++i]);
                config.sphereCenter.y = std::stof(argv[++i]);
                config.sphereCenter.z = std::stof(argv[++i]);
                config.sphereRadius = std::stof(argv[++i]);
            } else if (arg == "--plane") {
                if (i + 6 >= argc) {
                    throw std::runtime_error("--plane requires 6 values: PX PY PZ NX NY NZ");
                }
                config.hasPlane = true;
                config.planePoint.x = std::stof(argv[++i]);
                config.planePoint.y = std::stof(argv[++i]);
                config.planePoint.z = std::stof(argv[++i]);
                config.planeNormal.x = std::stof(argv[++i]);
                config.planeNormal.y = std::stof(argv[++i]);
                config.planeNormal.z = std::stof(argv[++i]);
            } else if (arg == "--frames") {
                needArg(i);
                config.frameCount = std::stoi(argv[++i]);
            } else if (arg == "--fps") {
                needArg(i);
                config.fps = std::stof(argv[++i]);
            } else if (arg == "--output") {
                needArg(i);
                config.outputPath = argv[++i];
            } else {
                std::cerr << "Unknown argument: " << arg << "\n";
                return ParseResult::Error;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n";
        return ParseResult::Error;
    }

    return ParseResult::Success;
}

} // namespace

int main(int argc, char** argv) {
    clothsim::SceneConfig config;
    switch (parseArgs(argc, argv, config)) {
        case ParseResult::ShowHelp:
            printUsage();
            return 0;
        case ParseResult::Error:
            printUsage();
            return 1;
        case ParseResult::Success:
            break;
    }

    clothsim::ClothMesh cloth(config.width, config.height, config.resX, config.resY);

    if (config.center) {
        for (clothsim::Particle& p : cloth.particles()) {
            p.position.x -= config.width * 0.5f;
            p.position.z -= config.height * 0.5f;
        }
    }
    if (config.offset != glm::vec3(0.0f)) {
        for (clothsim::Particle& p : cloth.particles()) {
            p.position += config.offset;
        }
    }
    for (clothsim::Particle& p : cloth.particles()) {
        p.previousPosition = p.position;
    }

    switch (config.pinMode) {
        case clothsim::PinMode::None:
            break;
        case clothsim::PinMode::TopRow:
            for (int x = 0; x < cloth.resolutionX(); ++x) {
                clothsim::Particle& p = cloth.particles()[cloth.particleIndex(x, 0)];
                p.pinned = true;
                p.invMass = 0.0f;
            }
            break;
        case clothsim::PinMode::LeftColumn:
            for (int y = 0; y < cloth.resolutionY(); ++y) {
                clothsim::Particle& p = cloth.particles()[cloth.particleIndex(0, y)];
                p.pinned = true;
                p.invMass = 0.0f;
            }
            break;
    }

    clothsim::CollisionWorld world;
    if (config.hasSphere) {
        world.addSphere(clothsim::SphereCollider{config.sphereCenter, config.sphereRadius});
    }
    if (config.hasPlane) {
        world.addPlane(clothsim::PlaneCollider{config.planePoint, glm::normalize(config.planeNormal)});
    }

    clothsim::SolverParams params;
    params.gravity = glm::vec3(0.0f, -config.gravity, 0.0f);
    params.wind = config.wind;
    params.damping = config.damping;
    params.constraintIterations = config.constraintIterations;
    params.selfCollisionDistance = config.selfCollisionDistance;

    clothsim::Solver solver(params);
    solver.setCollisionWorld(&world); // harmless even with zero colliders; needed for self-collision

    const float dt = 1.0f / config.fps;
    const int vertexCount = static_cast<int>(cloth.particles().size());

    std::cout << "cloth-sim: " << vertexCount << " particles, " << cloth.springs().size() << " springs\n";

    clothsim::PC2Writer writer(config.outputPath, vertexCount, /*startFrame=*/0.0f, config.fps);
    std::vector<glm::vec3> framePositions(vertexCount);

    const auto startTime = std::chrono::steady_clock::now();
    for (int frame = 0; frame < config.frameCount; ++frame) {
        solver.step(cloth, dt);

        for (int i = 0; i < vertexCount; ++i) {
            framePositions[i] = cloth.particles()[i].position;
        }
        writer.writeFrame(framePositions);
    }
    writer.finalize();
    const auto endTime = std::chrono::steady_clock::now();

    const double solveSeconds = std::chrono::duration<double>(endTime - startTime).count();
    const double solveFramesPerSecond = solveSeconds > 0.0 ? config.frameCount / solveSeconds : 0.0;

    std::cout << "Simulated " << config.frameCount << " frames in " << solveSeconds << "s ("
              << solveFramesPerSecond << " solve frames/sec)\n";

    const std::uintmax_t headerBytes = 32;
    const std::uintmax_t expectedBytes =
        headerBytes + static_cast<std::uintmax_t>(config.frameCount) * static_cast<std::uintmax_t>(vertexCount) * 12;
    const std::uintmax_t actualBytes = std::filesystem::file_size(config.outputPath);

    std::cout << "Wrote " << config.outputPath << ": " << actualBytes << " bytes"
              << " (expected " << expectedBytes << ")\n";
    if (actualBytes != expectedBytes) {
        std::cerr << "WARNING: PC2 file size does not match header + frames*vertices*12 bytes!\n";
        return 1;
    }

    return 0;
}
