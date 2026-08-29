cmake_minimum_required(VERSION 3.16)
project(clothsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(CLOTHSIM_BUILD_TESTS "Build the Catch2 test suite" ON)

include(FetchContent)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
)
FetchContent_MakeAvailable(glm)

# Everything except main.cpp lives in a static library so the test suite can
# link against the exact same simulation code without pulling in main()
# (which would collide with Catch2's own main).
add_library(clothsim_lib STATIC
    src/ClothMesh.cpp
    src/Solver.cpp
    src/CollisionWorld.cpp
    src/SpatialHashGrid.cpp
    src/PC2Writer.cpp
    src/SceneConfig.cpp
)

target_include_directories(clothsim_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(clothsim_lib PUBLIC glm::glm)

add_executable(clothsim src/main.cpp)
target_link_libraries(clothsim PRIVATE clothsim_lib)

if(CLOTHSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
