
(cl:in-package :asdf)

(defsystem "leg_detector3-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "obstacles_pos_vel" :depends-on ("_package_obstacles_pos_vel"))
    (:file "_package_obstacles_pos_vel" :depends-on ("_package"))
    (:file "pos_vel" :depends-on ("_package_pos_vel"))
    (:file "_package_pos_vel" :depends-on ("_package"))
  ))