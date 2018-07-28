# Install script for directory: /home/dnc/okvis/build/ceres/src/ceres_external

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/dnc/okvis/build")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres" TYPE FILE FILES
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/rotation.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/numeric_diff_functor.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/crs_matrix.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/jet.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/conditioned_cost_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/numeric_diff_cost_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/autodiff_local_parameterization.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/sized_cost_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/cost_function_to_functor.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/autodiff_cost_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/solver.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/dynamic_autodiff_cost_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/gradient_checker.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/local_parameterization.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/loss_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/dynamic_numeric_diff_cost_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/cost_function.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/types.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/covariance.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/fpclassify.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/iteration_callback.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/ordered_groups.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/ceres.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/c_api.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/normal_prior.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/problem.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/manual_constructor.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/variadic_evaluate.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/reenable_warnings.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/autodiff.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/numeric_diff.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/disable_warnings.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/port.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/fixed_array.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/eigen.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/macros.h"
    "/home/dnc/okvis/build/ceres/src/ceres_external/include/ceres/internal/scoped_ptr.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES "/home/dnc/okvis/build/ceres/src/ceres_external-build/config/ceres/internal/config.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/Ceres/CeresTargets.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/Ceres/CeresTargets.cmake"
         "/home/dnc/okvis/build/ceres/src/ceres_external-build/CMakeFiles/Export/share/Ceres/CeresTargets.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/Ceres/CeresTargets-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/Ceres/CeresTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Ceres" TYPE FILE FILES "/home/dnc/okvis/build/ceres/src/ceres_external-build/CMakeFiles/Export/share/Ceres/CeresTargets.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Ceres" TYPE FILE FILES "/home/dnc/okvis/build/ceres/src/ceres_external-build/CMakeFiles/Export/share/Ceres/CeresTargets-release.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Ceres" TYPE FILE FILES
    "/home/dnc/okvis/build/ceres/src/ceres_external-build/CeresConfig.cmake"
    "/home/dnc/okvis/build/ceres/src/ceres_external-build/CeresConfigVersion.cmake"
    "/home/dnc/okvis/build/ceres/src/ceres_external/cmake/FindEigen.cmake"
    "/home/dnc/okvis/build/ceres/src/ceres_external/cmake/FindGlog.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/dnc/okvis/build/ceres/src/ceres_external-build/internal/ceres/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/dnc/okvis/build/ceres/src/ceres_external-build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/dnc/okvis/build/ceres/src/ceres_external-build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
