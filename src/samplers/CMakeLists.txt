add_library(SAMPLER_2D
            MinkowskiLibrarySamplerSE2.cpp
            MinkowskiSamplerSE2.cpp)
target_link_libraries(SAMPLER_2D
                      GEOM
                      UTIL)

add_library(SAMPLER_3D
            C3FGenerator3D.cpp
            MinkowskiSamplerSE3.cpp)
target_link_libraries(SAMPLER_3D
                      GEOM
                      UTIL)
