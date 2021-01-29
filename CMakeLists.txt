cmake_minimum_required(VERSION 3.10)

# Project's name
project(HighwayRoadMap LANGUAGES CXX)

# Specify the minimum version for CMake
set(CMAKE_CXX_STANDARD 14)
set(default_build_type "RelWithDebInfo")

# Specify Binary dir
set(CMAKE_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# Include directories
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/test)

#add_compile_options(
#        -Wall
#        -Wextra
#        -Woverloaded-virtual
#)

# Dependency packages

find_package(OMPL REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
link_libraries(${OMPL_LIBRARIES})

find_package(FCL REQUIRED)
include_directories(${FCL_INCLUDE_DIRS})
link_libraries(${FCL_LIBRARIES})

find_package(CCD REQUIRED)
include_directories(${CCD_INCLUDE_DIRS})
link_libraries(${CCD_LIBRARIES})

find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})
link_libraries(${CGAL_LIBRARIES})

find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIRS})
link_libraries(${GMP_LIBRARIES})

find_package(orocos_kdl)
include_directories(${orocos_kdl_INCLUDE_DIRS})
link_libraries(${orocos_kdl_LIBRARIES})

find_package(kdl_parser)
link_libraries(${kdl_parser_LIBRARIES})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
link_libraries(${GTEST_LIBRARIES})

# Subdirectories
add_subdirectory(src)
add_subdirectory(test)
