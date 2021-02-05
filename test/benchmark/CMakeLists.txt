add_executable(BenchOMPL3D BenchOMPL3D.cpp)
target_link_libraries(BenchOMPL3D
                      OMPL_PLANNER
                      GEOM
                      UTIL
                      TESTUTIL)

add_executable(BenchOMPL3DArticulated BenchOMPL3DArticulated.cpp)
target_link_libraries(BenchOMPL3DArticulated
                      OMPL_PLANNER
                      GEOM
                      UTIL
                      TESTUTIL)

#add_executable(BenchOMPL3DEllipsoid BenchOMPL3DEllipsoid.cpp)
#target_link_libraries(BenchOMPL3DEllipsoid
#                      UTIL
#                      HRM_3D
#                      OMPL_PLANNER
#                      GEOM
#                      TESTUTIL
#                      ${OMPL})

add_executable(BenchHighway3D BenchHighway3D.cpp)
target_link_libraries(BenchHighway3D
                      UTIL
                      HRM_3D
                      HIGHWAY_UTIL
                      GEOM
                      TESTUTIL
                      ${OMPL})

add_executable(BenchProbHRM3D BenchProbHRM3D.cpp)
target_link_libraries(BenchProbHRM3D
                      UTIL
                      HRM_3D
                      HIGHWAY_UTIL
                      GEOM
                      TESTUTIL
                      ${OMPL})
