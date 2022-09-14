add_executable(BenchOMPL3D BenchOMPL3D.cpp)
target_link_libraries(BenchOMPL3D
                      OMPLPlanner
                      TestUtil)

add_executable(BenchOMPL3DArticulated BenchOMPL3DArticulated.cpp)
target_link_libraries(BenchOMPL3DArticulated
                      OMPLPlanner
                      TestUtil)

add_executable(BenchHRM2D BenchHRM2D.cpp)
target_link_libraries(BenchHRM2D
                      HRM2D
                      TestUtil)

add_executable(BenchHRM3D BenchHRM3D.cpp)
target_link_libraries(BenchHRM3D
                      HRM3D
                      TestUtil)

add_executable(BenchProbHRM3D BenchProbHRM3D.cpp)
target_link_libraries(BenchProbHRM3D
                      HRM3D
                      TestUtil)

add_executable(BenchHRM3DAblation BenchHRM3DAblation.cpp)
target_link_libraries(BenchHRM3DAblation
                      HRM3D
                      TestUtil)

add_executable(BenchProbHRM3DAblation BenchProbHRM3DAblation.cpp)
target_link_libraries(BenchProbHRM3DAblation
                      HRM3D
                      TestUtil)
