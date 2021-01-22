add_library(SAMPLER_2D
            FreeSpaceSE2.cpp
            MinkowskiLibrarySamplerSE2.cpp
            MinkowskiSamplerSE2.cpp
            PlannerSE2.cpp)
target_link_libraries(SAMPLER_2D
                      GEOM
                      UTIL)

add_library(SAMPLER_3D
            MinkowskiSamplerSE3.cpp
            FreeSpaceSE3.cpp
            PlannerSE3.cpp
            Planner3DArticulated.cpp)
target_link_libraries(SAMPLER_3D
                      GEOM
                      UTIL)
