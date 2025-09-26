cmake_minimum_required(VERSION 3.16)

option(GBASP_DEBUG "Enable debug mode" OFF)

if(GBASP_DEBUG)
    message(STATUS "Debug mode enabled")
    add_compile_definitions(GBASP_DEBUG)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Project name
project(GBASP VERSION 1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add Dear ImGui sources
set(INCLUDES_DIR "${CMAKE_SOURCE_DIR}/includes")
set(IMGUI_DIR "${INCLUDES_DIR}/imgui")
set(IMGUI_BACKENDS_DIR "${IMGUI_DIR}/backends")
set(IMGUI_MEMORY_EDITOR_DIR "${INCLUDES_DIR}/imgui_club/imgui_memory_editor")

set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_BACKENDS_DIR}/imgui_impl_sdl3.cpp
    ${IMGUI_BACKENDS_DIR}/imgui_impl_sdlrenderer3.cpp
)

# Add source files
file(GLOB_RECURSE SOURCES src/*/*.cpp src/*/*.c src/*.cpp src/*.c)

# Create object library
add_library(${PROJECT_NAME}_obj OBJECT ${SOURCES} ${IMGUI_SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}_obj PRIVATE ${IMGUI_MEMORY_EDITOR_DIR})
target_include_directories(${PROJECT_NAME}_obj PRIVATE ${IMGUI_DIR})
target_include_directories(${PROJECT_NAME}_obj PRIVATE ${IMGUI_BACKENDS_DIR})


# Find SDL3 package
find_package(SDL3 REQUIRED CONFIG REQUIRED COMPONENTS SDL3-shared)
find_package(OpenGL REQUIRED)


# Enable warnings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE -Wall -Wextra -Werror -Wno-bool-operation)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE /W4)
endif()

# Add executable and link object files
add_executable(${PROJECT_NAME} $<TARGET_OBJECTS:${PROJECT_NAME}_obj>)
target_link_libraries(${PROJECT_NAME}_obj PRIVATE SDL3::SDL3 OpenGL::GL)
target_link_libraries(${PROJECT_NAME} PRIVATE SDL3::SDL3 OpenGL::GL)

