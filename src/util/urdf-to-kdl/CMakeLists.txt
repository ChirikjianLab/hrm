project(KDL_Parser)
cmake_minimum_required(VERSION 3.5)

file(GLOB KDL_Parser_SOURCES src/kdl_parser/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Boost REQUIRED COMPONENTS system)
IF (Boost_FOUND)
  include_directories(BEFORE
                      SYSTEM ${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIR})
ENDIF()

find_package(urdfdom REQUIRED)
IF (urdfdom_FOUND)
  include_directories(BEFORE
                      SYSTEM ${urdfdom_INCLUDE_DIRS})
  link_directories(${urdfdom_LIBRARY_DIR})
ENDIF()


find_library(TinyXML NAMES tinyxml)

add_library(${PROJECT_NAME} STATIC ${KDL_Parser_SOURCES})
target_link_libraries(${PROJECT_NAME}
                      ${orocos_kdl_LIBRARIES}
                      ${TINYXML_LIBRARIES}
                      ${TinyXML}
                      ${Boost_LIBRARIES}
                      ${urdfdom_LIBRARIES})

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
CACHE INTERNAL
    "${PROJECT_NAME}: Include Directories" FORCE)

IF(BUILD_TEST)
    message(STATUS "Building with test executable")
    add_executable(test ${PROJECT_SOURCE_DIR}/test/check_kdl_parser.cpp)
    target_link_libraries(test ${PROJECT_NAME})
ENDIF()
