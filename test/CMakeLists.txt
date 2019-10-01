# 2D version
add_executable(test_HighwayRoadMap2D test.cpp)
target_link_libraries(test_HighwayRoadMap2D UTIL
                                            GEOM
                                            SOURCE_2D
                                            ${SOURCE_2D})

#add_executable(test_PLANNER_ACS_2D PLANNER_test_ACS_2D.cpp)
#target_link_libraries(test_PLANNER_ACS_2D ${OMPL})

#add_executable(test_PLANNER_FCL_2D PLANNER_test_FCL_2D.cpp)
#target_link_libraries(test_PLANNER_FCL_2D ${OMPL}
#                                          ${FCL_LIBRARIES}
#                                          ${CCD_LIBRARIES})

#add_executable(test_PLANNER_FCL_TGL_2D PLANNER_test_FCL_TGL_2D.cpp)
#target_link_libraries(test_PLANNER_FCL_TGL_2D ${OMPL}
#                                              ${FCL_LIBRARIES}
#                                              ${CCD_LIBRARIES})

# 3D version
add_executable(highwayRoadMap3D test_highway3d.cpp)
target_link_libraries(highwayRoadMap3D UTIL
                                       SOURCE_3D
                                       GEOM
                                       ${FCL_LIBRARIES}
                                       ${CCD_LIBRARIES})

#add_executable(test_PLANNER_FCL_3D PLANNER_test_FCL_3D.cpp)
#target_link_libraries(test_PLANNER_FCL_3D ${UTIL}
#                                          ${SOURCE_3D}
#                                          ${OMPL}
#                                          ${FCL_LIBRARIES}
#                                          ${CCD_LIBRARIES})

add_executable(HRM3D_multiBody test_hrm3d_multibody.cpp)
target_link_libraries(HRM3D_multiBody UTIL
                                      SOURCE_3D
                                      GEOM
                                      ${FCL_LIBRARIES}
                                      ${CCD_LIBRARIES})

# Subdirectory
add_subdirectory(sampler_test)
