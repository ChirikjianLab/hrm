add_library(HRM_2D
            HighwayRoadMap.cpp
            HighwayRoadMap2d.cpp
            HRM2DMultiBody.cpp)
target_link_libraries(HRM_2D
                      GEOM
                      UTIL)

add_library(HRM_3D
            HighwayRoadMap3d.cpp
            Hrm3dMultiBody.cpp
            Hrm3dMultiBodyAdaptive.cpp)
target_link_libraries(HRM_3D
                      GEOM
                      UTIL)
