FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/dwa_local_planner_moving_obs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/dwa_local_planner_moving_obs/msg/__init__.py"
  "../src/dwa_local_planner_moving_obs/msg/_OdometryMovingObstacles.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
