list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/util/external/kdl_parser/cmake)

find_package(urdfdom REQUIRED)
IF (urdfdom_FOUND)
  include_directories(BEFORE
                      SYSTEM ${urdfdom_INCLUDE_DIRS})
  link_directories(${urdfdom_LIBRARY_DIR})
ENDIF()


find_package(TinyXML2 REQUIRED)

file(GLOB KDL_Parser_SOURCES src/kdl_parser/*.cpp)

include_directories(include)

add_library(KDL_Parser STATIC ${KDL_Parser_SOURCES})
target_link_libraries(KDL_Parser
                      ${orocos_kdl_LIBRARIES}
                      ${TinyXML2_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${urdfdom_LIBRARIES})

set(KDL_Parser_INCLUDE_DIRS
    include
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    CACHE INTERNAL
    "KDL_Parser: Include Directories" FORCE)