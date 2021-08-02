add_executable(BenchOMPL3D BenchOMPL3D.cpp)
target_link_libraries(BenchOMPL3D
                      OMPL_PLANNER
                      TEST_UTIL)

add_executable(BenchOMPL3DArticulated BenchOMPL3DArticulated.cpp)
target_link_libraries(BenchOMPL3DArticulated
                      OMPL_PLANNER
                      TEST_UTIL)

#add_executable(BenchOMPL3DEllipsoid BenchOMPL3DEllipsoid.cpp)
#target_link_libraries(BenchOMPL3DEllipsoid
#                      UTIL
#                      HRM_3D
#                      OMPL_PLANNER
#                      GEOM
#                      TESTUTIL
#                      ${OMPL})

add_executable(BenchHRM2D BenchHRM2D.cpp)
target_link_libraries(BenchHRM2D
                      HRM_2D
                      TEST_UTIL)

add_executable(BenchHRM3D BenchHighway3D.cpp)
target_link_libraries(BenchHRM3D
                      HRM_3D
                      TEST_UTIL)

add_executable(BenchProbHRM3D BenchProbHRM3D.cpp)
target_link_libraries(BenchProbHRM3D
                      HRM_3D
                      TEST_UTIL)
