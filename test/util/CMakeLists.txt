add_library(TestUtil
            DisplayPlanningData.cpp
            GTestUtils.cpp
            ParsePlanningSettings.cpp)
target_link_libraries(TestUtil
                      Geometry)
