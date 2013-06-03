FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/leg_detector3/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/obstacles_pos_vel.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_obstacles_pos_vel.lisp"
  "../msg_gen/lisp/pos_vel.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_pos_vel.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
