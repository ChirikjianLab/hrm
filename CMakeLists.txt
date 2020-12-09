cmake_minimum_required(VERSION 3.10)

# Project's name
project(highwayRoadMap LANGUAGES CXX)

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
IF (OMPL_FOUND)
    include_directories(${OMPL_INCLUDE_DIRS})
    link_libraries(${OMPL_LIBRARIES})
    ADD_DEFINITIONS( "-DHAS_OMPL" )
ENDIF()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(FCL REQUIRED)
if(FCL_FOUND)
  include_directories(${FCL_INCLUDE_DIRS})
  link_libraries(${FCL_LIBRARIES})
  add_definitions("-DHAS_FCL")
endif()

find_package(Boost REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  add_definitions("-DHAS_BOOST")
endif()

find_package(CCD REQUIRED)
if(CCD_FOUND)
  include_directories(${CDD_INCLUDE_DIRS})
  link_libraries(${CCD_LIBRARIES})
  add_definitions("-DHAS_CCD")
endif()

find_package(CGAL REQUIRED)
if(CGAL_FOUND)
  include_directories(${CGAL_INCLUDE_DIRS})
  link_libraries(${CGAL_LIBRARIES})
  add_definitions("-DHAS_CGAL")
endif()

find_package(GMP REQUIRED)
if(GMP_FOUND)
  include_directories(${GMP_INCLUDE_DIRS})
  link_libraries(${GMP_LIBRARIES})
  add_definitions("-DHAS_GMP")
endif()

find_package(GTest REQUIRED)
if(GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
  link_libraries(${GTEST_LIBRARIES})
  add_definitions("-DHAS_GTEST")
endif()

# Subdirectories
add_subdirectory(src)
add_subdirectory(test)
