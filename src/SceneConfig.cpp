#include "clothsim/SceneConfig.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace clothsim {

namespace {

PinMode parsePinMode(const std::string& value, int lineNumber, const std::string& path) {
    if (value == "none") return PinMode::None;
    if (value == "top_row") return PinMode::TopRow;
    if (value == "left_column") return PinMode::LeftColumn;
    throw std::runtime_error("SceneConfig: unknown pin_mode '" + value + "' at " + path + ":" + std::to_string(lineNumber));
}

} // namespace

SceneConfig loadSceneConfig(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("SceneConfig: failed to open scene file: " + path);
    }

    SceneConfig config;
    std::string rawLine;
    int lineNumber = 0;

    while (std::getline(file, rawLine)) {
        ++lineNumber;

        const std::size_t hashPos = rawLine.find('#');
        const std::string line = hashPos == std::string::npos ? rawLine : rawLine.substr(0, hashPos);

        std::istringstream iss(line);
        std::string key;
        if (!(iss >> key)) {
            continue; // blank or comment-only line
        }

        auto fail = [&](const std::string& reason) {
            throw std::runtime_error("SceneConfig: " + reason + " for key '" + key + "' at " + path + ":" +
                                      std::to_string(lineNumber));
        };
        auto readFloat = [&]() {
            float v;
            if (!(iss >> v)) fail("expected a number");
            return v;
        };
        auto readInt = [&]() {
            int v;
            if (!(iss >> v)) fail("expected an integer");
            return v;
        };
        auto readString = [&]() {
            std::string v;
            if (!(iss >> v)) fail("expected a value");
            return v;
        };

        if (key == "width") {
            config.width = readFloat();
        } else if (key == "height") {
            config.height = readFloat();
        } else if (key == "resx") {
            config.resX = readInt();
        } else if (key == "resy") {
            config.resY = readInt();
        } else if (key == "iterations") {
            config.constraintIterations = readInt();
        } else if (key == "damping") {
            config.damping = readFloat();
        } else if (key == "gravity") {
            config.gravity = readFloat();
        } else if (key == "wind") {
            config.wind.x = readFloat();
            config.wind.y = readFloat();
            config.wind.z = readFloat();
        } else if (key == "self_collision") {
            config.selfCollisionDistance = readFloat();
        } else if (key == "pin_mode") {
            config.pinMode = parsePinMode(readString(), lineNumber, path);
        } else if (key == "center") {
            const std::string v = readString();
            config.center = (v == "true" || v == "1");
        } else if (key == "offset") {
            config.offset.x = readFloat();
            config.offset.y = readFloat();
            config.offset.z = readFloat();
        } else if (key == "sphere") {
            config.hasSphere = true;
            config.sphereCenter.x = readFloat();
            config.sphereCenter.y = readFloat();
            config.sphereCenter.z = readFloat();
            config.sphereRadius = readFloat();
        } else if (key == "plane") {
            config.hasPlane = true;
            config.planePoint.x = readFloat();
            config.planePoint.y = readFloat();
            config.planePoint.z = readFloat();
            config.planeNormal.x = readFloat();
            config.planeNormal.y = readFloat();
            config.planeNormal.z = readFloat();
        } else if (key == "frames") {
            config.frameCount = readInt();
        } else if (key == "fps") {
            config.fps = readFloat();
        } else if (key == "output") {
            config.outputPath = readString();
        } else {
            throw std::runtime_error("SceneConfig: unknown key '" + key + "' at " + path + ":" + std::to_string(lineNumber));
        }
    }

    return config;
}

} // namespace clothsim
