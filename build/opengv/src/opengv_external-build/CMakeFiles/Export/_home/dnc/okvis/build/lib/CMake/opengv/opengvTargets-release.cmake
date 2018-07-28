#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "opengv" for configuration "Release"
set_property(TARGET opengv APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opengv PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/home/dnc/okvis/build/lib/libopengv.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS opengv )
list(APPEND _IMPORT_CHECK_FILES_FOR_opengv "/home/dnc/okvis/build/lib/libopengv.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
