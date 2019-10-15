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
if(FCL_FOUND)
  include_directories(${FCL_INCLUDE_DIR})
  link_libraries(${FCL_LIBRARIES})
  add_definitions("-DHAS_FCL")
endif()

find_package(Boost REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIR})
  add_definitions("-DHAS_BOOST")
endif()

find_package(CCD REQUIRED)
if(CCD_FOUND)
  include_directories(${CDD_INCLUDE_DIR})
  link_libraries(${CCD_LIBRARIES})
  add_definitions("-DHAS_CCD")
endif()

find_package(CGAL REQUIRED)
if(CGAL_FOUND)
  include_directories(${CGAL_INCLUDE_DIR})
  link_libraries(${CGAL_LIBRARIES})
  add_definitions("-DHAS_CGAL")
endif()

find_package(GMP REQUIRED)
if(GMP_FOUND)
  include_directories(${GMP_INCLUDE_DIR})
  link_libraries(${GMP_LIBRARIES})
  add_definitions("-DHAS_GMP")
endif()

# Subdirectories
add_subdirectory(src)
add_subdirectory(test)
# add_subdirectory(benchmark)
