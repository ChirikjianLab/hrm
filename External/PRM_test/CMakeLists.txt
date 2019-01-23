# Specify the minimum version for CMake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 3.0)

# Project's name
project(PRM_test)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")

# Packages
find_library(OMPL_LIB ompl REQUIRED)
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
set(HEADER_FILE_ONLY /include/geometry/SuperEllipse.h)
set(HEADER_FILE_ONLY /include/geometry/SuperEllipse3D.h)
set(HEADER_FILE_ONLY /include/collision/ACScollision.h)
set(HEADER_FILE_ONLY /include/collision/ACScollision3D.h)
set(HEADER_FILE_ONLY /include/collision/FCLcollision.h)
set(HEADER_FILE_ONLY /include/collision/FCLcollision3D.h)
set(HEADER_FILE_ONLY /include/tester/ACStester.h)
set(HEADER_FILE_ONLY /include/tester/ACStester3D.h)
set(HEADER_FILE_ONLY /include/tester/FCLtester.h)
set(HEADER_FILE_ONLY /include/tester/FCLtester3D.h)
set(HEADER_FILE_ONLY /include/tester/FCLTGLtester.h)
set(HEADER_FILE_ONLY /include/util/FileReader.h)
set(HEADER_FILE_ONLY /inclde/util/RotationUtil.h)

set(HEADER_FILE_ONLY /include/tester/Tester2D.h)
set(HEADER_FILE_ONLY /include/tester/Tester3D.h)
set(HEADER_FILE_ONLY /include/tester/ACSmbtester.h)

set(HEADER_FILE_ONLY /include/util/plannerUtil.h)

# Sources
set(SOURCE
    "${PROJECT_SOURCE_DIR}/src/geometry/SuperEllipse.cpp"
    "${PROJECT_SOURCE_DIR}/src/geometry/SuperEllipse3D.cpp"
    "${PROJECT_SOURCE_DIR}/src/collision/ACScollision.cpp"
    "${PROJECT_SOURCE_DIR}/src/collision/FCLcollision.cpp"
    "${PROJECT_SOURCE_DIR}/src/collision/ACScollision3D.cpp"
    "${PROJECT_SOURCE_DIR}/src/collision/FCLcollision3D.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/ACStester.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/ACStester3D.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/FCLtester.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/FCLtester3D.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/FCLTGLtester.cpp"
    "${PROJECT_SOURCE_DIR}/src/util/FileReader.cpp"
    "${PROJECT_SOURCE_DIR}/src/util/RotationUtil.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/Tester2D.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/Tester3D.cpp"
    "${PROJECT_SOURCE_DIR}/src/tester/ACSmbtester.cpp"
    )

# Executables
add_library(tester ${SOURCE})

add_executable(test_PLANNER_ACS_2D ${PROJECT_SOURCE_DIR}/src/PLANNER_test_ACS_2D.cpp)
target_link_libraries(test_PLANNER_ACS_2D tester "${OMPL_LIB}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

add_executable(test_PLANNER_FCL_2D ${PROJECT_SOURCE_DIR}/src/PLANNER_test_FCL_2D.cpp)
target_link_libraries(test_PLANNER_FCL_2D tester "${OMPL_LIB}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

add_executable(test_PLANNER_FCL_TGL_2D ${PROJECT_SOURCE_DIR}/src/PLANNER_test_FCL_TGL_2D.cpp)
target_link_libraries(test_PLANNER_FCL_TGL_2D tester "${OMPL_LIB}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

add_executable(test_PLANNER_ACS_mb2D ${PROJECT_SOURCE_DIR}/src/PLANNER_test_ACS_mb2D.cpp)
target_link_libraries(test_PLANNER_ACS_mb2D tester "${OMPL_LIB}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

add_executable(test_PLANNER_ACS_3D ${PROJECT_SOURCE_DIR}/src/PLANNER_test_ACS_3D.cpp)
target_link_libraries(test_PLANNER_ACS_3D tester "${OMPL_LIB}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

add_executable(test_PLANNER_FCL_3D ${PROJECT_SOURCE_DIR}/src/PLANNER_test_FCL_3D.cpp)
target_link_libraries(test_PLANNER_FCL_3D tester "${OMPL_LIB}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

add_executable(test_sampler ${PROJECT_SOURCE_DIR}/src/sampler_test.cpp)
target_link_libraries(test_sampler "${OMPL_LIB}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")


