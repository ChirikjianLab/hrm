# Specify the minimum version for CMake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 3.0)

# Project's name
project(highwayRoadMap)

set(CMAKE_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/benchmark)

# Packages
find_library(OMPL REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(FCL REQUIRED)
IF (FCL_FOUND)
    include_directories(${FCL_INCLUDE_DIR})
    link_libraries(${FCL_LIBRARIES})
    ADD_DEFINITIONS( "-DHAS_FCL" )
ENDIF()

FIND_PACKAGE(Boost REQUIRED)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

find_package(CCD REQUIRED)
IF (CCD_FOUND)
    INCLUDE_DIRECTORIES(${CDD_INCLUDE_DIR})
    link_libraries(${CCD_LIBRARIES})
    ADD_DEFINITIONS( "-DHAS_CCD" )
ENDIF()

find_package(CGAL REQUIRED)
IF (CGAL_FOUND)
    INCLUDE_DIRECTORIES(${CGAL_INCLUDE_DIR})
    link_libraries(${CGAL_LIBRARIES})
    ADD_DEFINITIONS( "-DHAS_CGAL" )
ENDIF()

find_package(GMP REQUIRED)
IF (GMP_FOUND)
    INCLUDE_DIRECTORIES(${GMP_INCLUDE_DIR})
    link_libraries(${GMP_LIBRARIES})
    ADD_DEFINITIONS( "-DHAS_GMP" )
ENDIF()

# Subdirectories
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(benchmark)
