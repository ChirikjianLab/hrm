# Unit tests
# Geometry: SuperEllipse, SuperQuadrics, TightlyFittedEllipsoid
add_executable(GTestGeometry GTestGeometry.cpp)
target_link_libraries(GTestGeometry
                      UTIL
                      GEOM)
