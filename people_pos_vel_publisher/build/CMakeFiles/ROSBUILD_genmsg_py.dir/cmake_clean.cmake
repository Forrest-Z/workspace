FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/people_pos_vel_publisher/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/people_pos_vel_publisher/msg/__init__.py"
  "../src/people_pos_vel_publisher/msg/_people_pos_vel.py"
  "../src/people_pos_vel_publisher/msg/_pos_vel.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
