add_library(SOURCE_2D HighwayRoadMap.cpp HighwayRoadMap2d.cpp)
target_link_libraries(SOURCE_2D GEOM UTIL)

add_library(SOURCE_3D
            HighwayRoadMap3d.cpp
            Hrm3dMultiBody.cpp
            Hrm3dMultiBodyAdaptive.cpp)
target_link_libraries(SOURCE_3D GEOM UTIL)
