# Specify the minimum version for CMake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 0.1)

# Project's name
project(HighwayRoadMap)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}")

# Packages
# find_package(OpenCV REQUIRED)

# Headers
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/geometry/superellipse.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/planners/interval.h)

# Executables
add_executable(HighwayRoadMap ${PROJECT_SOURCE_DIR}/test/test.cpp)
add_executable(HighwayRoadMap ${PROJECT_SOURCE_DIR}/src/planners/interval.cpp)
#add_executable(HighwayRoadMap ${PROJECT_SOURCE_DIR}/test/prm.cpp)
add_executable(HighwayRoadMap ${PROJECT_SOURCE_DIR}/src/geometry/superellipse.cpp)
add_executable(HighwayRoadMap ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.cpp)

# TARGET_LINK_LIBRARIES(HighwayRoadMap ${OpenCV_LIBS})
