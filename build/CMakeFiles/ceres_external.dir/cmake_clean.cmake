FILE(REMOVE_RECURSE
  "CMakeFiles/ceres_external"
  "CMakeFiles/ceres_external-complete"
  "ceres/src/ceres_external-stamp/ceres_external-install"
  "ceres/src/ceres_external-stamp/ceres_external-mkdir"
  "ceres/src/ceres_external-stamp/ceres_external-download"
  "ceres/src/ceres_external-stamp/ceres_external-update"
  "ceres/src/ceres_external-stamp/ceres_external-patch"
  "ceres/src/ceres_external-stamp/ceres_external-configure"
  "ceres/src/ceres_external-stamp/ceres_external-build"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ceres_external.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
