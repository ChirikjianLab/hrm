add_subdirectory(datastructure)
add_subdirectory(geometry)
add_subdirectory(planners)
add_subdirectory(util)
add_subdirectory(test/util)

# Install main library
install(TARGETS DataStructure
                Geometry
                HighwayRoadMap
                HRM2D
                HRM3D
                OMPLPlanner
                Util
                TestUtil
        EXPORT HighwayRoadMapTargets
        LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_PROJECT_NAME}")
