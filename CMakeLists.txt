# TODO : the versions included here should be explicitly tested out,preferably in CI
cmake_minimum_required(VERSION 3.16...3.18)

# Project's name
project(HighwayRoadMap VERSION 0.2.2 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
set(default_build_type "Release")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/static_analysis)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/sanitizer)

# Include directories
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/test)

find_package(static_analysis)
#find_package(sanitizer)

#add_compile_options(
#        -Wall
#        -Wextra
#        -Woverloaded-virtual
#)

# Dependency packages

find_package(OMPL REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
link_libraries(${OMPL_LIBRARIES})

find_package(FCL 0.6 REQUIRED)
include_directories(${FCL_INCLUDE_DIRS})
link_libraries(${FCL_LIBRARIES})

find_package(CCD REQUIRED)
include_directories(${CCD_INCLUDE_DIRS})
link_libraries(${CCD_LIBRARIES})

find_package(CGAL 5.0 REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})

find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIRS})
link_libraries(${GMP_LIBRARIES})

find_package(orocos_kdl)
include_directories(${orocos_kdl_INCLUDE_DIRS})
link_libraries(${orocos_kdl_LIBRARIES})

# kdl_parser is OPTIONAL for now, we have provided a lite-version of KDL_parser
find_package(kdl_parser QUIET)
if(kdl_parser_FOUND)
    message(STATUS "Found KDL_Parser from ROS dependencies")
    link_libraries(${kdl_parser_LIBRARIES})
    include_directories(${kdl_parser_INCLUDE_DIRS})
else()
    message(STATUS "ROS version of KDL_Parser NOT FOUND, will use pure CMake version")
    add_subdirectory(${PROJECT_SOURCE_DIR}/src/util/external/kdl_parser)
endif()

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Boost 1.71.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Threads)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
link_libraries(${GTEST_LIBRARIES})

# Subdirectories
add_subdirectory(src)

# Copy planning config files to build dir
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/demo/
          ${CMAKE_CURRENT_SOURCE_DIR}/resources/SO3_sequence/q_icosahedron_60.csv
          ${CMAKE_CURRENT_SOURCE_DIR}/resources/3D/urdf/snake.urdf
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test/config/)

set(BUILD_TESTING "" ON)
enable_testing()
add_subdirectory(test)
