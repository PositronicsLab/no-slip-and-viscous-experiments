
# Umfpack lib usually requires linking to a blas library.
# It is up to the user of this module to find a BLAS and link to it.

if (SUPERLU_INCLUDES AND SUPERLU_LIBRARIES)
  set(SUPERLU_FIND_QUIETLY TRUE)
endif (SUPERLU_INCLUDES AND SUPERLU_LIBRARIES)

find_path(SUPERLU_INCLUDES
  NAMES
  supermatrix.h
  PATHS
  ${SUPERLU_PATH}/include
  $ENV{SUPERLUDIR}/include
  ${INCLUDE_INSTALL_DIR}
  PATH_SUFFIXES
  superlu
)

find_library(SUPERLU_LIBRARIES 
  NAMES
  superlu
  superlu_4.3 
  PATHS 
  ${SUPERLU_PATH}/lib
  $ENV{SUPERLUDIR}/lib 
  ${LIB_INSTALL_DIR})
  
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SUPERLU DEFAULT_MSG
                                  SUPERLU_INCLUDES SUPERLU_LIBRARIES)

mark_as_advanced(SUPERLU_INCLUDES SUPERLU_LIBRARIES)

