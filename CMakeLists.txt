cmake_minimum_required(VERSION 3.16)
project(clothsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(clothsim
    src/main.cpp
)

target_include_directories(clothsim PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)
