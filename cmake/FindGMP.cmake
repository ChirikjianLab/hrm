# Try to find the GMP librairies GMP_FOUND - system has GMP lib GMP_INCLUDE_DIRS
# - the GMP include directory GMP_LIBRARIES - Libraries needed to use GMP

if(GMP_INCLUDE_DIRS AND GMP_LIBRARIES)
  # Already in cache, be silent
  set(GMP_FIND_QUIETLY TRUE)
endif(GMP_INCLUDE_DIRS AND GMP_LIBRARIES)

find_path(GMP_INCLUDE_DIRS NAMES gmp.h PATHS $ENV{GMP_INC})
find_library(GMP_LIBRARIES NAMES gmp libgmp PATHS $ENV{GMP_LIB})
find_library(GMPXX_LIBRARIES NAMES gmpxx libgmpxx PATHS $ENV{GMP_LIB})
# MESSAGE(STATUS "GMP libs: " ${GMP_LIBRARIES} " " ${GMPXX_LIBRARIES} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GMP
                                  DEFAULT_MSG
                                  GMP_INCLUDE_DIRS
                                  GMP_LIBRARIES)

mark_as_advanced(GMP_INCLUDE_DIRS GMP_LIBRARIES)
