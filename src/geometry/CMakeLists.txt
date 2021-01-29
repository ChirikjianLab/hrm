add_library(GEOM
            FreeSpace2D.cpp
            FreeSpace3D.cpp
            SuperEllipse.cpp
            SuperQuadrics.cpp
            TightFitEllipsoid.cpp)
target_link_libraries(GEOM
                      UTIL)
