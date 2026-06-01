cmake_minimum_required(VERSION 3.16)
project(PhysicsEngine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(include)

# Source files for the static library
set(LIB_SOURCES
    src/AABB.cpp
    src/BVH.cpp
    src/PhysicsWorld.cpp
    src/ConstraintSolver.cpp
)

# Static library target
add_library(physics_engine STATIC ${LIB_SOURCES})
target_include_directories(physics_engine PUBLIC include)

# Installation rules
install(TARGETS physics_engine ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
