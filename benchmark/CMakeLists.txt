add_executable(benchmark3D_OMPL benchmark_3d_ompl.cpp)
target_link_libraries(benchmark3D_OMPL ${UTIL}
                                       ${SOURCE_3D}
                                       ${BENCH}
                                       ${OMPL}
                                       ${FCL_LIBRARIES}
                                       ${CCD_LIBRARIES})

add_executable(benchmark3D_HIGHWAY benchmark_3d_highway.cpp)
target_link_libraries(benchmark3D_HIGHWAY ${UTIL}
                                          ${SOURCE_3D}
                                          ${BENCH}
                                          ${OMPL}
                                          ${FCL_LIBRARIES}
                                          ${CCD_LIBRARIES})

add_executable(benchmark3D_HRM_MULTIBODY benchmark_3d_hrm_multibody.cpp)
target_link_libraries(benchmark3D_HRM_MULTIBODY ${UTIL}
                                                ${SOURCE_3D}
                                                ${BENCH}
                                                ${OMPL}
                                                ${FCL_LIBRARIES}
                                                ${CCD_LIBRARIES})

add_executable(benchmark3D_HRM_MULTI_ADAPTIVE benchmark_3d_hrm_multi_adaptive.cpp)
target_link_libraries(benchmark3D_HRM_MULTI_ADAPTIVE ${UTIL}
                                                     ${SOURCE_3D}
                                                     ${BENCH}
                                                     ${OMPL}
                                                     ${FCL_LIBRARIES}
                                                     ${CCD_LIBRARIES})

add_subdirectory(highway)
add_subdirectory(ompl)
