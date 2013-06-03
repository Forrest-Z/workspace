FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ptu_control/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/PtuGotoAction.msg"
  "../msg/PtuGotoGoal.msg"
  "../msg/PtuGotoActionGoal.msg"
  "../msg/PtuGotoResult.msg"
  "../msg/PtuGotoActionResult.msg"
  "../msg/PtuGotoFeedback.msg"
  "../msg/PtuGotoActionFeedback.msg"
  "../msg/PtuSetVelAction.msg"
  "../msg/PtuSetVelGoal.msg"
  "../msg/PtuSetVelActionGoal.msg"
  "../msg/PtuSetVelResult.msg"
  "../msg/PtuSetVelActionResult.msg"
  "../msg/PtuSetVelFeedback.msg"
  "../msg/PtuSetVelActionFeedback.msg"
  "../msg/PtuResetAction.msg"
  "../msg/PtuResetGoal.msg"
  "../msg/PtuResetActionGoal.msg"
  "../msg/PtuResetResult.msg"
  "../msg/PtuResetActionResult.msg"
  "../msg/PtuResetFeedback.msg"
  "../msg/PtuResetActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
