; Auto-generated. Do not edit!


(cl:in-package leg_detector3-msg)


;//! \htmlinclude obstacles_pos_vel.msg.html

(cl:defclass <obstacles_pos_vel> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (number_of_obstacles
    :reader number_of_obstacles
    :initarg :number_of_obstacles
    :type cl:integer
    :initform 0)
   (pos_vel
    :reader pos_vel
    :initarg :pos_vel
    :type (cl:vector leg_detector3-msg:pos_vel)
   :initform (cl:make-array 20 :element-type 'leg_detector3-msg:pos_vel :initial-element (cl:make-instance 'leg_detector3-msg:pos_vel))))
)

(cl:defclass obstacles_pos_vel (<obstacles_pos_vel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacles_pos_vel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacles_pos_vel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_detector3-msg:<obstacles_pos_vel> is deprecated: use leg_detector3-msg:obstacles_pos_vel instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <obstacles_pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:header-val is deprecated.  Use leg_detector3-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'number_of_obstacles-val :lambda-list '(m))
(cl:defmethod number_of_obstacles-val ((m <obstacles_pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:number_of_obstacles-val is deprecated.  Use leg_detector3-msg:number_of_obstacles instead.")
  (number_of_obstacles m))

(cl:ensure-generic-function 'pos_vel-val :lambda-list '(m))
(cl:defmethod pos_vel-val ((m <obstacles_pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:pos_vel-val is deprecated.  Use leg_detector3-msg:pos_vel instead.")
  (pos_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacles_pos_vel>) ostream)
  "Serializes a message object of type '<obstacles_pos_vel>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'number_of_obstacles)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pos_vel))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacles_pos_vel>) istream)
  "Deserializes a message object of type '<obstacles_pos_vel>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number_of_obstacles) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'pos_vel) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'pos_vel)))
    (cl:dotimes (i 20)
    (cl:setf (cl:aref vals i) (cl:make-instance 'leg_detector3-msg:pos_vel))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacles_pos_vel>)))
  "Returns string type for a message object of type '<obstacles_pos_vel>"
  "leg_detector3/obstacles_pos_vel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacles_pos_vel)))
  "Returns string type for a message object of type 'obstacles_pos_vel"
  "leg_detector3/obstacles_pos_vel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacles_pos_vel>)))
  "Returns md5sum for a message object of type '<obstacles_pos_vel>"
  "643e95694865b2bdc861ebe92204cffe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacles_pos_vel)))
  "Returns md5sum for a message object of type 'obstacles_pos_vel"
  "643e95694865b2bdc861ebe92204cffe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacles_pos_vel>)))
  "Returns full string definition for message of type '<obstacles_pos_vel>"
  (cl:format cl:nil "#An array of position and velocities of moving obstacles~%Header header~%int32 number_of_obstacles~%leg_detector3/pos_vel[20] pos_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: leg_detector3/pos_vel~%# A representation of position and velocity of object~%Header header~%float32 pos_x~%float32 pos_y~%float32 pos_z~%float32 vel_x~%float32 vel_y~%float32 vel_z~%int32 class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacles_pos_vel)))
  "Returns full string definition for message of type 'obstacles_pos_vel"
  (cl:format cl:nil "#An array of position and velocities of moving obstacles~%Header header~%int32 number_of_obstacles~%leg_detector3/pos_vel[20] pos_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: leg_detector3/pos_vel~%# A representation of position and velocity of object~%Header header~%float32 pos_x~%float32 pos_y~%float32 pos_z~%float32 vel_x~%float32 vel_y~%float32 vel_z~%int32 class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacles_pos_vel>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos_vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacles_pos_vel>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacles_pos_vel
    (cl:cons ':header (header msg))
    (cl:cons ':number_of_obstacles (number_of_obstacles msg))
    (cl:cons ':pos_vel (pos_vel msg))
))
