add_executable(BenchOMPL3D BenchOMPL3D.cpp)
target_link_libraries(BenchOMPL3D
                      UTIL
                      HRM_3D
                      OMPL_UTIL
                      GEOM
                      TESTUTIL
                      ${OMPL}
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES})

add_executable(BenchOMPL3DEllipsoid BenchOMPL3DEllipsoid.cpp)
target_link_libraries(BenchOMPL3DEllipsoid
                      UTIL
                      HRM_3D
                      OMPL_UTIL
                      GEOM
                      TESTUTIL
                      ${OMPL})

add_executable(BenchHighway3D BenchHighway3D.cpp)
target_link_libraries(BenchHighway3D
                      UTIL
                      HRM_3D
                      HIGHWAY_UTIL
                      GEOM
                      TESTUTIL
                      ${OMPL})

add_executable(BenchHRM3DAdaptive
               BenchHRM3DAdaptive.cpp)
target_link_libraries(BenchHRM3DAdaptive
                      UTIL
                      HRM_3D
                      HIGHWAY_UTIL
                      GEOM
                      TESTUTIL
                      ${OMPL})
