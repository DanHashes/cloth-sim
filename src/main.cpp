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
#include "clothsim/Solver.hpp"

namespace {

struct CliConfig {
    float width = 2.0f;
    float height = 2.0f;
    int resX = 20;
    int resY = 20;

    int constraintIterations = 5;
    float damping = 0.98f;
    float gravity = 9.81f;
    glm::vec3 wind{0.0f, 0.0f, 0.0f};
    float selfCollisionDistance = 0.0f;
    bool pinTop = true;

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

void printUsage() {
    std::cout <<
        "cloth-sim: cloth physics simulator with PC2 point-cache export\n\n"
        "Usage: clothsim [options]\n"
        "  --width F                 cloth width in world units (default 2.0)\n"
        "  --height F                cloth height in world units (default 2.0)\n"
        "  --resx N                  particles across (default 20)\n"
        "  --resy N                  particles down (default 20)\n"
        "  --iterations N            constraint relaxation passes per step (default 5)\n"
        "  --damping F               velocity retained per step, 0-1 (default 0.98)\n"
        "  --gravity F               gravity magnitude along -Y (default 9.81)\n"
        "  --wind X Y Z              constant wind acceleration (default 0 0 0)\n"
        "  --self-collision F        minimum particle separation, 0 disables (default 0)\n"
        "  --no-pin-top              do not pin the top row (free-falling sheet)\n"
        "  --sphere CX CY CZ R       add a sphere collider\n"
        "  --plane PX PY PZ NX NY NZ add a plane collider\n"
        "  --frames N                number of frames to simulate (default 300)\n"
        "  --fps F                   simulation frame rate (default 60)\n"
        "  --output PATH             output .pc2 path (default cloth_output.pc2)\n"
        "  --help                    show this message\n";
}

enum class ParseResult { Success, ShowHelp, Error };

ParseResult parseArgs(int argc, char** argv, CliConfig& config) {
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
            } else if (arg == "--no-pin-top") {
                config.pinTop = false;
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
    CliConfig config;
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

    if (config.pinTop) {
        for (int x = 0; x < cloth.resolutionX(); ++x) {
            clothsim::Particle& p = cloth.particles()[cloth.particleIndex(x, 0)];
            p.pinned = true;
            p.invMass = 0.0f;
        }
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
