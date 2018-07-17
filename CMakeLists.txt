# Specify the minimum version for CMake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 0.1)

# Project's name
project(highwayRoadMap)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}")

# Packages
# find_package(Eigen3)
find_library(OMPL_TEST ompl REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

# Headers
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/geometry/superellipse.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/planners/interval.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/geometry/ptinpoly.h)

# Executables
add_executable(highwayRoadMap ${PROJECT_SOURCE_DIR}/test/test.cpp)

add_executable(test_prm ${PROJECT_SOURCE_DIR}/test/PRM_test.cpp)
target_link_libraries(test_prm "${OMPL_TEST}")

add_executable(highwayRoadMap ${PROJECT_SOURCE_DIR}/src/planners/interval.cpp)
add_executable(highwayRoadMap ${PROJECT_SOURCE_DIR}/src/geometry/ptinpoly.cpp)
add_executable(highwayRoadMap ${PROJECT_SOURCE_DIR}/src/geometry/superellipse.cpp)
add_executable(highwayRoadMap ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.cpp)

add_executable(test_prm ${PROJECT_SOURCE_DIR}/src/planners/interval.cpp)
add_executable(test_prm ${PROJECT_SOURCE_DIR}/src/geometry/ptinpoly.cpp)
add_executable(test_prm ${PROJECT_SOURCE_DIR}/src/geometry/superellipse.cpp)
add_executable(test_prm ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.cpp)
# TARGET_LINK_LIBRARIES(highwayRoadMap ${OpenCV_LIBS})
