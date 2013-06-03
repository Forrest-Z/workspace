; Auto-generated. Do not edit!


(cl:in-package leg_detector3-msg)


;//! \htmlinclude pos_vel.msg.html

(cl:defclass <pos_vel> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0)
   (pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:float
    :initform 0.0)
   (vel_x
    :reader vel_x
    :initarg :vel_x
    :type cl:float
    :initform 0.0)
   (vel_y
    :reader vel_y
    :initarg :vel_y
    :type cl:float
    :initform 0.0)
   (vel_z
    :reader vel_z
    :initarg :vel_z
    :type cl:float
    :initform 0.0)
   (class
    :reader class
    :initarg :class
    :type cl:integer
    :initform 0))
)

(cl:defclass pos_vel (<pos_vel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos_vel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos_vel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_detector3-msg:<pos_vel> is deprecated: use leg_detector3-msg:pos_vel instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:header-val is deprecated.  Use leg_detector3-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:pos_x-val is deprecated.  Use leg_detector3-msg:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:pos_y-val is deprecated.  Use leg_detector3-msg:pos_y instead.")
  (pos_y m))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:pos_z-val is deprecated.  Use leg_detector3-msg:pos_z instead.")
  (pos_z m))

(cl:ensure-generic-function 'vel_x-val :lambda-list '(m))
(cl:defmethod vel_x-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:vel_x-val is deprecated.  Use leg_detector3-msg:vel_x instead.")
  (vel_x m))

(cl:ensure-generic-function 'vel_y-val :lambda-list '(m))
(cl:defmethod vel_y-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:vel_y-val is deprecated.  Use leg_detector3-msg:vel_y instead.")
  (vel_y m))

(cl:ensure-generic-function 'vel_z-val :lambda-list '(m))
(cl:defmethod vel_z-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:vel_z-val is deprecated.  Use leg_detector3-msg:vel_z instead.")
  (vel_z m))

(cl:ensure-generic-function 'class-val :lambda-list '(m))
(cl:defmethod class-val ((m <pos_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_detector3-msg:class-val is deprecated.  Use leg_detector3-msg:class instead.")
  (class m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos_vel>) ostream)
  "Serializes a message object of type '<pos_vel>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'class)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos_vel>) istream)
  "Deserializes a message object of type '<pos_vel>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'class) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos_vel>)))
  "Returns string type for a message object of type '<pos_vel>"
  "leg_detector3/pos_vel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_vel)))
  "Returns string type for a message object of type 'pos_vel"
  "leg_detector3/pos_vel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos_vel>)))
  "Returns md5sum for a message object of type '<pos_vel>"
  "13172b912205b204b582bfc1dd2711b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos_vel)))
  "Returns md5sum for a message object of type 'pos_vel"
  "13172b912205b204b582bfc1dd2711b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos_vel>)))
  "Returns full string definition for message of type '<pos_vel>"
  (cl:format cl:nil "# A representation of position and velocity of object~%Header header~%float32 pos_x~%float32 pos_y~%float32 pos_z~%float32 vel_x~%float32 vel_y~%float32 vel_z~%int32 class~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos_vel)))
  "Returns full string definition for message of type 'pos_vel"
  (cl:format cl:nil "# A representation of position and velocity of object~%Header header~%float32 pos_x~%float32 pos_y~%float32 pos_z~%float32 vel_x~%float32 vel_y~%float32 vel_z~%int32 class~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos_vel>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos_vel>))
  "Converts a ROS message object to a list"
  (cl:list 'pos_vel
    (cl:cons ':header (header msg))
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
    (cl:cons ':pos_z (pos_z msg))
    (cl:cons ':vel_x (vel_x msg))
    (cl:cons ':vel_y (vel_y msg))
    (cl:cons ':vel_z (vel_z msg))
    (cl:cons ':class (class msg))
))
