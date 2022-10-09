# 3D version
add_executable(DemoHRMRigidBodyPlanning3D DemoHRMRigidBodyPlanning3D.cpp)
target_link_libraries(DemoHRMRigidBodyPlanning3D
                      HRM3D
                      TestUtil)

add_executable(DemoOMPLRigidBodyPlanning3D DemoOMPLRigidBodyPlanning3D.cpp)
target_link_libraries(DemoOMPLRigidBodyPlanning3D
                      OMPLPlanner
                      TestUtil)
