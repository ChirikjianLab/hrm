add_library(Util
            DistanceMetric.cpp
            EllipsoidSeparation.cpp
            EllipsoidSQCollisionFCL.cpp
            ExponentialFunction.cpp
            InterpolateSE3.cpp
            ParseURDF.cpp
            Parse2dCsvFile.cpp)
if(kdl_parser_FOUND)
    target_link_libraries(Util
                          ${kdl_parser_LIBRARIES})
else()
    target_link_libraries(Util
                          KDL_Parser)
    target_include_directories(Util PUBLIC external/kdl_parser/include)
endif()
