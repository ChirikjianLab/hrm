add_library(GEOM
            SuperEllipse.cpp
            SuperQuadrics.cpp
            TightFitEllipsoid.cpp)
target_link_libraries(GEOM
                      UTIL)
