add_library(HighwayRoadMap HighwayRoadMap.cpp)
target_link_libraries(HighwayRoadMap
                      DataStructure
                      Geometry
                      ${OMPL_LIBRARIES})

add_library(HRM2D
            HRM2D.cpp
            HRM2DKC.cpp)
target_link_libraries(HRM2D
                      HighwayRoadMap)

add_library(HRM3D
            HRM3D.cpp
            ProbHRM3D.cpp)
target_link_libraries(HRM3D
                      HighwayRoadMap)

add_subdirectory(ompl_interface)
