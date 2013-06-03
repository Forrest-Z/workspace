FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/leg_detector3/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/leg_detector3/msg/__init__.py"
  "../src/leg_detector3/msg/_obstacles_pos_vel.py"
  "../src/leg_detector3/msg/_pos_vel.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
