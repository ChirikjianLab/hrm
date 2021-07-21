# Unit tests
enable_testing()

# Geometry: SuperEllipse, SuperQuadrics, TightlyFittedEllipsoid
add_executable(TestGeometry TestGeometry.cpp)
target_link_libraries(TestGeometry
                      UTIL
                      GEOM
                      ${GTEST_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT})
add_test(TestGeometry ${EXECUTABLE_OUTPUT_PATH}/TestGeometry)

# Planners                      
# 2D version
add_executable(TestHRM2D TestHRM2D.cpp)
target_link_libraries(TestHRM2D
                      UTIL
                      GEOM
                      HRM_2D
                      TESTUTIL
                      ${GTEST_LIBRARIES})
add_test(TestHRM2D ${EXECUTABLE_OUTPUT_PATH}/TestHRM2D)

# 3D version
add_executable(TestHRM3D TestHRM3D.cpp)
target_link_libraries(TestHRM3D
                      UTIL
                      HRM_3D
                      GEOM
                      TESTUTIL
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES}
                      ${GTEST_LIBRARIES})
add_test(TestHRM3D ${EXECUTABLE_OUTPUT_PATH}/TestHRM3D)

add_subdirectory(benchmark)
add_subdirectory(util)
