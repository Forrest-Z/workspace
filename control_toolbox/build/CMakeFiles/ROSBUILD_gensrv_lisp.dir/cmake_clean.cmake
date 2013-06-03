FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/control_toolbox/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/SetPidGains.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetPidGains.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
