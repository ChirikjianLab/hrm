# 2D version
add_executable(TestHRM2D TestHRM2D.cpp)
target_link_libraries(TestHRM2D
                      UTIL
                      GEOM
                      SOURCE_2D
                      TESTUTIL)

add_executable(TestHRM2DTightFittedEllipse TestHRM2DTightFittedEllipse.cpp)
target_link_libraries(TestHRM2DTightFittedEllipse
                      UTIL
                      GEOM
                      SOURCE_2D
                      TESTUTIL)

add_executable(TestHRM2DMultiBody TestHRM2DMultiBody.cpp)
target_link_libraries(TestHRM2DMultiBody
                      UTIL
                      GEOM
                      SOURCE_2D
                      TESTUTIL)

# 3D version
add_executable(TestHRM3D TestHRM3D.cpp)
target_link_libraries(TestHRM3D
                      UTIL
                      SOURCE_3D
                      GEOM
                      TESTUTIL
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES})

add_executable(TestHRM3DMultiBody TestHRM3DMultiBody.cpp)
target_link_libraries(TestHRM3DMultiBody
                      UTIL
                      SOURCE_3D
                      GEOM
                      TESTUTIL
                      ${FCL_LIBRARIES}
                      ${CCD_LIBRARIES})

add_subdirectory(googletest)
add_subdirectory(util)
