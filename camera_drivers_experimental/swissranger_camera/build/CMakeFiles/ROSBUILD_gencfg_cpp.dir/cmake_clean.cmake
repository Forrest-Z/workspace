FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/swissranger_camera/SwissRangerConfig.h"
  "../docs/SwissRangerConfig.dox"
  "../docs/SwissRangerConfig-usage.dox"
  "../src/swissranger_camera/cfg/SwissRangerConfig.py"
  "../docs/SwissRangerConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
