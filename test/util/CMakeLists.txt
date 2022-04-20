add_library(TEST_UTIL
            DisplayPlanningData.cpp
            ParsePlanningSettings.cpp)
target_link_libraries(TEST_UTIL
                      GEOM
                      UTIL)
