find_program(CLANG_TIDY_COMMAND NAMES clang-tidy-12 DOC "Path to clang-tidy executable")

if(NOT CLANG_TIDY_COMMAND)
    message(FATAL_ERROR "clang tidy executable NOT found")
else()
    message(STATUS "Found Clang Tidy: ${CLANG_TIDY_COMMAND}")
    set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_COMMAND})
endif()