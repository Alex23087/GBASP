cmake_minimum_required(VERSION 3.16)

# Project name
project(GBASP VERSION 1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE SOURCES src/*/*.cpp src/*/*.c src/*.cpp src/*.c)
file(GLOB_RECURSE QT_SOURCES src/UI/*/*.cpp src/UI/*/*.c src/UI/*.cpp src/UI/*.c)

# Create object library
add_library(${PROJECT_NAME}_obj OBJECT ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}_obj PRIVATE include)

# Find Qt6
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

# Set Qt6 link libraries
target_link_libraries(${PROJECT_NAME}_obj PRIVATE Qt6::Core Qt6::Widgets)

# Standard Qt project setup
qt_standard_project_setup()

# Create a Qt library for UI sources
qt_add_library(${PROJECT_NAME}_ui STATIC ${QT_SOURCES})

# Enable warnings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE -Wall -Wextra)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE /W4)
endif()

# Add executable and link object files
add_executable(${PROJECT_NAME} $<TARGET_OBJECTS:${PROJECT_NAME}_obj>)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_ui Qt6::Core Qt6::Widgets)