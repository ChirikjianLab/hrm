add_library(TESTUTIL
            ParsePlanningSettings.cpp)

target_link_libraries(TESTUTIL
                      GEOM
                      UTIL)

add_library(HIGHWAY_UTIL
            highway_planner.cpp
            UtilProbHRM.cpp)

add_library(OMPL_UTIL
            ompl_planner.cpp
            ompl_planner_ellipsoid.cpp)
