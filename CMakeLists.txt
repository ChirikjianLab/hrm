# Specify the minimum version for CMake
cmake_minimum_required(VERSION 0.1)

# Project's name
project(HighwayRoadMap)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}")

# Headers
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/geometry/minkowski/minkowski_es_2d.h)

# Executables
add_executable(HighwayRoadMap ${PROJECT_SOURCE_DIR}/test/test.cpp)
add_executable(HighwayRoadMap ${PROJECT_SOURCE_DIR}/src/geometry/minkowski/minkowski_es_2d.cpp)
