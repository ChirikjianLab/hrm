# 2D version
add_executable(TestHRM2D TestHRM2D.cpp)
target_link_libraries(TestHRM2D
                      UTIL
                      GEOM
                      HRM_2D
                      TESTUTIL)

# 3D version
add_executable(TestHRM3D TestHRM3D.cpp)
target_link_libraries(TestHRM3D
                      UTIL
                      HRM_3D
                      GEOM
                      TESTUTIL
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES})

add_executable(TestOMPL3D TestOMPL3D.cpp)
target_link_libraries(TestOMPL3D
                      TESTUTIL
                      OMPL_PLANNER)

add_executable(TestOMPL3DArticulated TestMinkSampler3D.cpp)
target_link_libraries(TestOMPL3DArticulated
                      TESTUTIL
                      OMPL_PLANNER)

add_subdirectory(benchmark)
add_subdirectory(gtest)
add_subdirectory(util)
