FILE(REMOVE_RECURSE
  "CMakeFiles/brisk_external"
  "CMakeFiles/brisk_external-complete"
  "brisk/src/brisk_external-stamp/brisk_external-install"
  "brisk/src/brisk_external-stamp/brisk_external-mkdir"
  "brisk/src/brisk_external-stamp/brisk_external-download"
  "brisk/src/brisk_external-stamp/brisk_external-update"
  "brisk/src/brisk_external-stamp/brisk_external-patch"
  "brisk/src/brisk_external-stamp/brisk_external-configure"
  "brisk/src/brisk_external-stamp/brisk_external-build"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/brisk_external.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
