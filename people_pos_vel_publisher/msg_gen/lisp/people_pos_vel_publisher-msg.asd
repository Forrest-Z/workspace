
(cl:in-package :asdf)

(defsystem "people_pos_vel_publisher-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "people_pos_vel" :depends-on ("_package_people_pos_vel"))
    (:file "_package_people_pos_vel" :depends-on ("_package"))
    (:file "pos_vel" :depends-on ("_package_pos_vel"))
    (:file "_package_pos_vel" :depends-on ("_package"))
  ))