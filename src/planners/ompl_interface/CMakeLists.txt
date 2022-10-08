add_library(OMPLPlanner
            OMPL2D.cpp
            OMPL3D.cpp
            OMPL3DArticulated.cpp)
target_link_libraries(OMPLPlanner
                      DataStructure
                      Geometry)
