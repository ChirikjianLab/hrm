add_executable(BenchOMPL3D BenchOMPL3D.cpp)
target_link_libraries(BenchOMPL3D
                      UTIL
                      SOURCE_3D
                      OMPL_BENCH
                      GEOM
                      TESTUTIL
                      ${OMPL}
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES})

add_executable(BenchOMPL3DEllipsoid BenchOMPL3DEllipsoid.cpp)
target_link_libraries(BenchOMPL3DEllipsoid
                      UTIL
                      SOURCE_3D
                      OMPL_BENCH
                      GEOM
                      TESTUTIL
                      ${OMPL})

add_executable(BenchHRM3D BenchHRM3D.cpp)
target_link_libraries(BenchHRM3D
                      UTIL
                      SOURCE_3D
                      HIGHWAY_BENCH
                      GEOM
                      TESTUTIL
                      ${OMPL})

add_executable(BenchHRM3DMultiBody BenchHRM3DMultiBody.cpp)
target_link_libraries(BenchHRM3DMultiBody
                      UTIL
                      SOURCE_3D
                      HIGHWAY_BENCH
                      GEOM
                      TESTUTIL
                      ${OMPL})

add_executable(BenchHRM3DAdaptive
               BenchHRM3DAdaptive.cpp)
target_link_libraries(BenchHRM3DAdaptive
                      UTIL
                      SOURCE_3D
                      HIGHWAY_BENCH
                      GEOM
                      TESTUTIL
                      ${OMPL})

add_subdirectory(highway)
add_subdirectory(ompl)
