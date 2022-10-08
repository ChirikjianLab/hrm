# Unit tests
enable_testing()

add_library(TestUtil util/details/DisplayPlanningData.cpp
                     util/details/GTestUtils.cpp
                     util/details/ParsePlanningSettings.cpp)
target_link_libraries(TestUtil
                      Geometry
                      Util
                      ${GTEST_LIBRARIES})

# Geometry: SuperEllipse, SuperQuadrics, TightlyFittedEllipsoid
add_executable(TestGeometry TestGeometry.cpp)
target_link_libraries(TestGeometry
                      Geometry)
add_test(TestGeometry ${EXECUTABLE_OUTPUT_PATH}/TestGeometry)

# Planners                      
# 2D version
add_executable(TestHRM2D TestHRM2D.cpp)
target_link_libraries(TestHRM2D
                      HRM2D
                      TestUtil)
add_test(TestHRM2D ${EXECUTABLE_OUTPUT_PATH}/TestHRM2D)

# 3D version
add_executable(TestHRM3D TestHRM3D.cpp)
target_link_libraries(TestHRM3D
                      HRM3D
                      TestUtil)
add_test(TestHRM3D ${EXECUTABLE_OUTPUT_PATH}/TestHRM3D)

add_executable(TestProbHRM3D TestProbHRM3D.cpp)
target_link_libraries(TestProbHRM3D
                      HRM3D
                      TestUtil)
add_test(TestProbHRM3D ${EXECUTABLE_OUTPUT_PATH}/TestProbHRM3D)

# Ablated version
add_executable(TestHRM3DAblation TestHRM3DAblation.cpp)
target_link_libraries(TestHRM3DAblation
                      HRM3D
                      TestUtil)
add_test(TestHRM3DAblation ${EXECUTABLE_OUTPUT_PATH}/TestHRM3DAblation)

# OMPL
add_executable(TestOMPL3D TestOMPL3D.cpp)
target_link_libraries(TestOMPL3D
                      OMPLPlanner
                      TestUtil)
add_test(TestOMPL3D ${EXECUTABLE_OUTPUT_PATH}/TestOMPL3D)

add_executable(TestOMPL3DArticulated TestOMPL3DArticulated.cpp)
target_link_libraries(TestOMPL3DArticulated
                      OMPLPlanner
                      TestUtil)
add_test(TestOMPL3DArticulated ${EXECUTABLE_OUTPUT_PATH}/TestOMPL3DArticulated)

add_subdirectory(benchmark)

# Install test utils
file(GLOB_RECURSE test_util CONFIGURE_DEPENDS "util/*.h")
install(FILES ${test_util}
        DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${CMAKE_PROJECT_NAME}${INSTALL_SUFFIX}/test/util")

install(TARGETS ${TestUtil}
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_PROJECT_NAME}${INSTALL_SUFFIX}/test/util")
