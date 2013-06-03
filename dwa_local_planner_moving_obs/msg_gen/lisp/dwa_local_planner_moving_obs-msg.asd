
(cl:in-package :asdf)

(defsystem "dwa_local_planner_moving_obs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OdometryMovingObstacles" :depends-on ("_package_OdometryMovingObstacles"))
    (:file "_package_OdometryMovingObstacles" :depends-on ("_package"))
  ))