# Specify the minimum version for CMake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 3.0)

# Project's name
project(highwayRoadMap)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}")

# Packages
find_library(OMPL_TEST ompl REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(FCL REQUIRED)
IF (FCL_FOUND)
    include_directories(${FCL_INCLUDE_DIR})
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
    ADD_DEFINITIONS( "-DHAS_CCD" )
ENDIF()

# Headers
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/geometry/superellipse.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/planners/interval.h)
set(HEADER_FILE_ONLY ${PROJECT_SOURCE_DIR}/src/geometry/ptinpoly.h)

# Sources
set(SOURCE
    "${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.cpp"
    "${PROJECT_SOURCE_DIR}/src/planners/interval.cpp"
    "${PROJECT_SOURCE_DIR}/src/geometry/superellipse.cpp"
    "${PROJECT_SOURCE_DIR}/src/geometry/ptinpoly.cpp")

# Executables
add_executable(highwayRoadMap ${PROJECT_SOURCE_DIR}/test/test.cpp ${SOURCE})

add_executable(test_prm ${PROJECT_SOURCE_DIR}/test/PRM_test.cpp ${SOURCE})
target_link_libraries(test_prm "${OMPL_TEST}")
add_executable(test_FCL_Ellipsoid_2D ${PROJECT_SOURCE_DIR}/test/PRM_test_FCL_Ellipsoid_2D.cpp)
target_link_libraries(test_FCL_Ellipsoid_2D "${OMPL_TEST}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")
add_executable(test_FCL_SuperEllipsoid_2D ${PROJECT_SOURCE_DIR}/test/PRM_test_FCL_SuperEllipsoid_2D.cpp)
target_link_libraries(test_FCL_SuperEllipsoid_2D "${OMPL_TEST}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")


# TARGET_LINK_LIBRARIES(highwayRoadMap ${OpenCV_LIBS})
