add_executable(sam_PRM_ACS_2D SP_test_ACS_2D.cpp)
target_link_libraries(sam_PRM_ACS_2D ${OMPL})

add_executable(sam_PLANNER_FCL_2D SP_test_FCL_2D.cpp)
target_link_libraries(sam_PLANNER_FCL_2D
                      ${OMPL}
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES})

add_executable(sam_PLANNER_FCL_TGL_2D SP_test_FCL_TGL_2D.cpp)
target_link_libraries(sam_PLANNER_FCL_TGL_2D
                      ${OMPL}
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES})
