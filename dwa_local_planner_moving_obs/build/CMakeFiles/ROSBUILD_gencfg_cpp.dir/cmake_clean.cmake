FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/dwa_local_planner_moving_obs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/dwa_local_planner_moving_obs/DWAPlannerMovingObsConfig.h"
  "../docs/DWAPlannerMovingObsConfig.dox"
  "../docs/DWAPlannerMovingObsConfig-usage.dox"
  "../src/dwa_local_planner_moving_obs/cfg/DWAPlannerMovingObsConfig.py"
  "../docs/DWAPlannerMovingObsConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
