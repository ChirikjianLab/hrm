# Unit tests
enable_testing()

# Geometry: SuperEllipse, SuperQuadrics, TightlyFittedEllipsoid
add_executable(TestGeometry TestGeometry.cpp)
target_link_libraries(TestGeometry
                      UTIL
                      GEOM
                      ${GTEST_LIBRARIES})
add_test(TestGeometry ${EXECUTABLE_OUTPUT_PATH}/TestGeometry)

# Planners                      
# 2D version
add_executable(TestHRM2D TestHRM2D.cpp)
target_link_libraries(TestHRM2D
                      HRM2D
                      TEST_UTIL
                      ${GTEST_LIBRARIES})
add_test(TestHRM2D ${EXECUTABLE_OUTPUT_PATH}/TestHRM2D)

# 3D version
add_executable(TestHRM3D TestHRM3D.cpp)
target_link_libraries(TestHRM3D
                      HRM3D
                      TEST_UTIL
                      ${GTEST_LIBRARIES})
add_test(TestHRM3D ${EXECUTABLE_OUTPUT_PATH}/TestHRM3D)

add_executable(TestProbHRM3D TestProbHRM3D.cpp)
target_link_libraries(TestProbHRM3D
                      HRM3D
                      TEST_UTIL
                      ${GTEST_LIBRARIES})
add_test(TestProbHRM3D ${EXECUTABLE_OUTPUT_PATH}/TestProbHRM3D)

add_subdirectory(benchmark)
add_subdirectory(util)
