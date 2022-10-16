# 3D version
add_executable(DemoHRMRigidBodyPlanning3D DemoHRMRigidBodyPlanning3D.cpp)
target_link_libraries(DemoHRMRigidBodyPlanning3D
                      HRM3D
                      TestUtil)

add_executable(DemoProbHRMArticulatedBodyPlanning3D DemoProbHRMArticulatedBodyPlanning3D.cpp)
target_link_libraries(DemoProbHRMArticulatedBodyPlanning3D
                      HRM3D
                      TestUtil)

add_executable(DemoOMPLRigidBodyPlanning3D DemoOMPLRigidBodyPlanning3D.cpp)
target_link_libraries(DemoOMPLRigidBodyPlanning3D
                      OMPLPlanner
                      TestUtil)

add_executable(DemoOMPLArticulatedBodyPlanning3D DemoOMPLArticulatedBodyPlanning3D.cpp)
target_link_libraries(DemoOMPLArticulatedBodyPlanning3D
                      OMPLPlanner
                      TestUtil)
