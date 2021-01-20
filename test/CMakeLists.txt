# 2D version
add_executable(TestHRM2D TestHRM2D.cpp)
target_link_libraries(TestHRM2D
                      UTIL
                      GEOM
                      HRM_2D
                      TESTUTIL)

add_executable(TestHRM2DTightFittedEllipse TestHRM2DTightFittedEllipse.cpp)
target_link_libraries(TestHRM2DTightFittedEllipse
                      UTIL
                      GEOM
                      HRM_2D
                      TESTUTIL)

add_executable(TestHRM2DMultiBody TestHRM2DMultiBody.cpp)
target_link_libraries(TestHRM2DMultiBody
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

add_executable(TestPlanningSE3Articulated TestPlanningSE3Articulated.cpp)
target_link_libraries(TestPlanningSE3Articulated
                      TESTUTIL
                      SAMPLER_3D)

add_subdirectory(benchmark)
add_subdirectory(gtest)
add_subdirectory(util)
