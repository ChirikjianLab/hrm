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
find_library(OMPL ompl REQUIRED)
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

## Libraries
#set(UTIL ${PROJECT_SOURCE_DIR}/src/planners/interval.cpp
#         ${PROJECT_SOURCE_DIR}/src/geometry/ptinpoly.cpp
#         ${PROJECT_SOURCE_DIR}/src/util/parse2dcsvfile.cpp)

#set(SOURCE_2D ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap.cpp
#              ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap2d.cpp
#              ${PROJECT_SOURCE_DIR}/src/geometry/superellipse.cpp)

#set(SOURCE_3D ${PROJECT_SOURCE_DIR}/src/planners/highwayroadmap3d.cpp
#              ${PROJECT_SOURCE_DIR}/src/planners/hrm3d_multibody.cpp
#              ${PROJECT_SOURCE_DIR}/src/planners/hrm3d_multi_adaptive.cpp
#              ${PROJECT_SOURCE_DIR}/src/geometry/superquadrics.cpp
#              ${PROJECT_SOURCE_DIR}/src/geometry/intersectlinemesh3d.cpp
#              ${PROJECT_SOURCE_DIR}/src/geometry/multibodytree3d.cpp
#              ${PROJECT_SOURCE_DIR}/src/util/mesh_gen.cpp)

#set(BENCH ${PROJECT_SOURCE_DIR}/benchmark/highway/highway_planner.cpp
#          ${PROJECT_SOURCE_DIR}/benchmark/highway/highway_multibody.cpp
#          ${PROJECT_SOURCE_DIR}/benchmark/highway/hrm_multi_adaptive_planner.cpp
#          ${PROJECT_SOURCE_DIR}/benchmark/ompl/ompl_planner.cpp)

## Executables
#add_executable(highwayRoadMap2D ${PROJECT_SOURCE_DIR}/test/test.cpp ${UTIL} ${SOURCE_2D})

#add_executable(test_PLANNER_ACS_2D ${PROJECT_SOURCE_DIR}/test/PLANNER_test_ACS_2D.cpp)
#target_link_libraries(test_PLANNER_ACS_2D "${OMPL}")

#add_executable(test_PLANNER_FCL_2D ${PROJECT_SOURCE_DIR}/test/PLANNER_test_FCL_2D.cpp)
#target_link_libraries(test_PLANNER_FCL_2D "${OMPL}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

#add_executable(test_PLANNER_FCL_TGL_2D ${PROJECT_SOURCE_DIR}/test/PLANNER_test_FCL_TGL_2D.cpp)
#target_link_libraries(test_PLANNER_FCL_TGL_2D "${OMPL}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

## Sampler versions
#add_executable(sam_PRM_ACS_2D ${PROJECT_SOURCE_DIR}/test/sampler_test/SP_test_ACS_2D.cpp)
#target_link_libraries(sam_PRM_ACS_2D "${OMPL}")

#add_executable(sam_PLANNER_FCL_2D ${PROJECT_SOURCE_DIR}/test/sampler_test/SP_test_FCL_2D.cpp)
#target_link_libraries(sam_PLANNER_FCL_2D "${OMPL}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

#add_executable(sam_PLANNER_FCL_TGL_2D ${PROJECT_SOURCE_DIR}/test/sampler_test/SP_test_FCL_TGL_2D.cpp)
#target_link_libraries(sam_PLANNER_FCL_TGL_2D "${OMPL}" "${FCL_LIBRARIES}" "${CCD_LIBRARIES}")

## 3D version
## Executables
#add_executable(highwayRoadMap3D ${PROJECT_SOURCE_DIR}/test/test_highway3d.cpp ${UTIL} ${SOURCE_3D})
#target_link_libraries(highwayRoadMap3D ${FCL_LIBRARIES} ${CCD_LIBRARIES})

#add_executable(test_PLANNER_FCL_3D ${PROJECT_SOURCE_DIR}/test/PLANNER_test_FCL_3D.cpp ${UTIL} ${SOURCE_3D})
#target_link_libraries(test_PLANNER_FCL_3D ${OMPL} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

#add_executable(HRM3D_multiBody ${PROJECT_SOURCE_DIR}/test/test_hrm3d_multibody.cpp ${UTIL} ${SOURCE_3D})
#target_link_libraries(HRM3D_multiBody ${FCL_LIBRARIES} ${CCD_LIBRARIES})

## Benchmark
#add_executable(benchmark3D_OMPL ${PROJECT_SOURCE_DIR}/benchmark/benchmark_3d_ompl.cpp ${UTIL} ${SOURCE_3D} ${BENCH})
#target_link_libraries(benchmark3D_OMPL ${OMPL} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

#add_executable(benchmark3D_HIGHWAY ${PROJECT_SOURCE_DIR}/benchmark/benchmark_3d_highway.cpp ${UTIL} ${SOURCE_3D} ${BENCH})
#target_link_libraries(benchmark3D_HIGHWAY ${OMPL} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

#add_executable(benchmark3D_HRM_MULTIBODY ${PROJECT_SOURCE_DIR}/benchmark/benchmark_3d_hrm_multibody.cpp
#    ${UTIL} ${SOURCE_3D} ${BENCH})
#target_link_libraries(benchmark3D_HRM_MULTIBODY ${OMPL} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

#add_executable(benchmark3D_HRM_MULTI_ADAPTIVE ${PROJECT_SOURCE_DIR}/benchmark/benchmark_3d_hrm_multi_adaptive.cpp
#    ${UTIL} ${SOURCE_3D} ${BENCH})
#target_link_libraries(benchmark3D_HRM_MULTI_ADAPTIVE ${OMPL} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(benchmark)
