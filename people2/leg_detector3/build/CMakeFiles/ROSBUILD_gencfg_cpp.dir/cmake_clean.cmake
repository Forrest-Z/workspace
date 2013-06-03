FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/leg_detector3/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/leg_detector3/LegDetectorConfig.h"
  "../docs/LegDetectorConfig.dox"
  "../docs/LegDetectorConfig-usage.dox"
  "../src/leg_detector3/cfg/LegDetectorConfig.py"
  "../docs/LegDetectorConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
