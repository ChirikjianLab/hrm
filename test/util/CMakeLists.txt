add_library(TESTUTIL
            ParsePlanningSettings.cpp)

target_link_libraries(TESTUTIL
                      GEOM
                      UTIL)
