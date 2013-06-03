FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/leg_detector3/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/leg_detector3/obstacles_pos_vel.h"
  "../msg_gen/cpp/include/leg_detector3/pos_vel.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
