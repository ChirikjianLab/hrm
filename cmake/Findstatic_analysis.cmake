option(USE_CLANG_TIDY "Use clang-tidy" ON)
if(USE_CLANG_TIDY)
    include(cmake/static_analysis/clang_tidy.cmake)
endif()