if(NOT kdl_parser_FOUND)
    add_subdirectory(urdf-to-kdl)
endif()

add_library(UTIL
            EllipsoidSeparation.cpp
            EllipsoidSQCollisionFCL.cpp
            ExponentialFunction.cpp
            DistanceMetric.cpp
            InterpolateSE3.cpp
            Interval.cpp
            LineIntersection.cpp
            MeshGenerator.cpp
            MultiBodyTree2D.cpp
            MultiBodyTree3D.cpp
            Parse2dCsvFile.cpp
            ParseURDF.cpp
            PointInPoly.cpp)
if(kdl_parser_FOUND)
    target_link_libraries(UTIL
                          ${kdl_parser_LIBRARIES})
else()
    target_link_libraries(UTIL
                          KDL_Parser)
    target_include_directories(UTIL PUBLIC urdf-to-kdl/include)
endif()
