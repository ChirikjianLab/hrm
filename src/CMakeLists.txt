add_subdirectory(datastructure)
add_subdirectory(geometry)
add_subdirectory(planners)
add_subdirectory(util)

# Install main library
install(TARGETS DataStructure
                Geometry
                HighwayRoadMap
                HRM2D
                HRM3D
                OMPLPlanner
                Util
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_PROJECT_NAME}${INSTALL_SUFFIX}")
