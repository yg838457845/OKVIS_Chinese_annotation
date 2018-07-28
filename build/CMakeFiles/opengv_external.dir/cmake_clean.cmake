FILE(REMOVE_RECURSE
  "CMakeFiles/opengv_external"
  "CMakeFiles/opengv_external-complete"
  "opengv/src/opengv_external-stamp/opengv_external-install"
  "opengv/src/opengv_external-stamp/opengv_external-mkdir"
  "opengv/src/opengv_external-stamp/opengv_external-download"
  "opengv/src/opengv_external-stamp/opengv_external-update"
  "opengv/src/opengv_external-stamp/opengv_external-patch"
  "opengv/src/opengv_external-stamp/opengv_external-configure"
  "opengv/src/opengv_external-stamp/opengv_external-build"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/opengv_external.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
