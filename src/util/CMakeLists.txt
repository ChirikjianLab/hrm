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
    target_include_directories(Util PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/util/external/kdl_parser/include>
                                           $<INSTALL_INTERFACE:external/kdl_parser/include>)
    
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/util/external/kdl_parser/include/kdl_parser
            CONFIGURATIONS Release
            DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
            
    install(TARGETS KDL_Parser
            EXPORT hrmTargets
            DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/")
endif()
