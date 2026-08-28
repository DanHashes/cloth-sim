cmake_minimum_required(VERSION 3.16)
project(clothsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
)
FetchContent_MakeAvailable(glm)

add_executable(clothsim
    src/main.cpp
    src/ClothMesh.cpp
    src/Solver.cpp
    src/CollisionWorld.cpp
)

target_include_directories(clothsim PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(clothsim PUBLIC glm::glm)
