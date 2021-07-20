add_library(HighwayRoadMap HighwayRoadMap.cpp)
target_link_libraries(HighwayRoadMap
                      GEOM
                      UTIL
                      ${OMPL_LIBRARIES})

add_library(HRM_2D
            HighwayRoadMap2d.cpp
            HRM2DMultiBody.cpp)
target_link_libraries(HRM_2D
                      HighwayRoadMap)

add_library(HRM_3D
            HighwayRoadMap3d.cpp
            Hrm3dMultiBody.cpp
            ProbHRM3D.cpp)
target_link_libraries(HRM_3D
                      HighwayRoadMap)

add_subdirectory(ompl_interface)
