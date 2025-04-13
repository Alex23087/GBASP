cmake_minimum_required(VERSION 3.16)

# Project name
project(GBASP VERSION 1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE SOURCES SM83/*.cpp SM83/*.c)

# Create object library
add_library(${PROJECT_NAME}_obj OBJECT ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}_obj PRIVATE include)

# Enable warnings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE -Wall -Wextra)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE /W4)
endif()

# Add executable and link object files
add_executable(${PROJECT_NAME} $<TARGET_OBJECTS:${PROJECT_NAME}_obj>)