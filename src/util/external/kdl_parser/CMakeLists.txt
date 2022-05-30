find_package(urdfdom REQUIRED)
IF (urdfdom_FOUND)
  include_directories(BEFORE
                      SYSTEM ${urdfdom_INCLUDE_DIRS})
  link_directories(${urdfdom_LIBRARY_DIR})
ENDIF()


find_library(TinyXML2 NAMES tinyxml2)

file(GLOB KDL_Parser_SOURCES src/kdl_parser/*.cpp)

include_directories(include)

add_library(KDL_Parser STATIC ${KDL_Parser_SOURCES})
target_link_libraries(KDL_Parser
                      ${orocos_kdl_LIBRARIES}
                      ${TINYXML2_LIBRARIES}
                      ${TinyXML}
                      ${Boost_LIBRARIES}
                      ${urdfdom_LIBRARIES})

set(KDL_Parser_INCLUDE_DIRS
    include
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
CACHE INTERNAL
    "KDL_Parser: Include Directories" FORCE)
