cmake_minimum_required(VERSION 3.16)

# Project name
project(GBASP VERSION 1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE SOURCES src/*/*.cpp src/*/*.c src/*.cpp src/*.c)

# Create object library
add_library(${PROJECT_NAME}_obj OBJECT ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}_obj PRIVATE include)


# Platform-dependent settings
if(APPLE)
    # Find Homebrew Qt installation
    set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt/6.9.0/lib/cmake" CACHE PATH "Path to Qt installed via Homebrew")
    # Alternative for newer Homebrew installations or Apple Silicon Macs
    if(NOT EXISTS ${CMAKE_PREFIX_PATH})
        set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt/6.9.0/lib/cmake" CACHE PATH "Path to Qt installed via Homebrew")
    endif()
elseif(UNIX)
    # Linux-specific settings
    find_package(Qt6 COMPONENTS Core REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
elseif(WIN32)
    # Windows-specific settings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT")
    set(CMAKE_PREFIX_PATH "C:/Qt/6.9.0/msvc2019_64/lib/cmake" CACHE PATH "Path to Qt installed via Qt Installer")
    # Alternative for newer Qt installations
    if(NOT EXISTS ${CMAKE_PREFIX_PATH})
        set(CMAKE_PREFIX_PATH "C:/Qt/6.9.0/msvc2019_64/lib/cmake" CACHE PATH "Path to Qt installed via Qt Installer")
    endif()
endif()
# Find Qt6
find_package(Qt6 COMPONENTS Core REQUIRED)
# Add Qt6Core as a dependency
target_link_libraries(${PROJECT_NAME}_obj PRIVATE Qt6::Core)


# Enable warnings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE -Wall -Wextra)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(${PROJECT_NAME}_obj PRIVATE /W4)
endif()

# Add executable and link object files
add_executable(${PROJECT_NAME} $<TARGET_OBJECTS:${PROJECT_NAME}_obj>)