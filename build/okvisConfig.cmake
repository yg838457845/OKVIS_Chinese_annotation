# - Config file for the OKVIS package
# It defines the following variables
#  OKVIS_INCLUDE_DIRS - include directories for FooBar
#  OKVIS_LIBRARIES    - libraries to link against
#  OKVIS_EXECUTABLE   - the okvis_app_synchronous executable
#  OKVIS_CERES_CONFIG - path to CeresConfig.cmake, to use find_package(ceres)

set(OKVIS_CERES_CONFIG "/home/dnc/okvis/build/share/Ceres/")
 
# Compute paths
get_filename_component(OKVIS_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(OKVIS_INCLUDE_DIRS "/home/dnc/okvis/okvis_util/include;/home/dnc/okvis/okvis_kinematics/include;/home/dnc/okvis/okvis_time/include;/home/dnc/okvis/okvis_cv/include;/home/dnc/okvis/okvis_common/include;/home/dnc/okvis/okvis_ceres/include;/home/dnc/okvis/okvis_timing/include;/home/dnc/okvis/okvis_matcher/include;/home/dnc/okvis/okvis_frontend/include;/home/dnc/okvis/okvis_multisensor_processing/include;/home/dnc/okvis/build")
 
# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET okvis AND NOT OKVIS_BINARY_DIR)
  include("${OKVIS_CMAKE_DIR}/okvisTargets.cmake")
endif()
 
# These are IMPORTED targets created by okvisTargets.cmake
set(OKVIS_LIBRARIES 
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv 
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_matcher
    okvis_frontend 
    okvis_multisensor_processing )
set(OKVIS_EXECUTABLE okvis_app_synchronous)
