; Auto-generated. Do not edit!


(cl:in-package dwa_local_planner_moving_obs-msg)


;//! \htmlinclude OdometryMovingObstacles.msg.html

(cl:defclass <OdometryMovingObstacles> (roslisp-msg-protocol:ros-message)
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
   (odom
    :reader odom
    :initarg :odom
    :type (cl:vector nav_msgs-msg:Odometry)
   :initform (cl:make-array 0 :element-type 'nav_msgs-msg:Odometry :initial-element (cl:make-instance 'nav_msgs-msg:Odometry))))
)

(cl:defclass OdometryMovingObstacles (<OdometryMovingObstacles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OdometryMovingObstacles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OdometryMovingObstacles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dwa_local_planner_moving_obs-msg:<OdometryMovingObstacles> is deprecated: use dwa_local_planner_moving_obs-msg:OdometryMovingObstacles instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <OdometryMovingObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dwa_local_planner_moving_obs-msg:header-val is deprecated.  Use dwa_local_planner_moving_obs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'number_of_obstacles-val :lambda-list '(m))
(cl:defmethod number_of_obstacles-val ((m <OdometryMovingObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dwa_local_planner_moving_obs-msg:number_of_obstacles-val is deprecated.  Use dwa_local_planner_moving_obs-msg:number_of_obstacles instead.")
  (number_of_obstacles m))

(cl:ensure-generic-function 'odom-val :lambda-list '(m))
(cl:defmethod odom-val ((m <OdometryMovingObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dwa_local_planner_moving_obs-msg:odom-val is deprecated.  Use dwa_local_planner_moving_obs-msg:odom instead.")
  (odom m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OdometryMovingObstacles>) ostream)
  "Serializes a message object of type '<OdometryMovingObstacles>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'number_of_obstacles)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'odom))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'odom))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OdometryMovingObstacles>) istream)
  "Deserializes a message object of type '<OdometryMovingObstacles>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number_of_obstacles) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'odom) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'odom)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'nav_msgs-msg:Odometry))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OdometryMovingObstacles>)))
  "Returns string type for a message object of type '<OdometryMovingObstacles>"
  "dwa_local_planner_moving_obs/OdometryMovingObstacles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OdometryMovingObstacles)))
  "Returns string type for a message object of type 'OdometryMovingObstacles"
  "dwa_local_planner_moving_obs/OdometryMovingObstacles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OdometryMovingObstacles>)))
  "Returns md5sum for a message object of type '<OdometryMovingObstacles>"
  "b75dd1fa87c85e47ac20e03fdf58c4da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OdometryMovingObstacles)))
  "Returns md5sum for a message object of type 'OdometryMovingObstacles"
  "b75dd1fa87c85e47ac20e03fdf58c4da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OdometryMovingObstacles>)))
  "Returns full string definition for message of type '<OdometryMovingObstacles>"
  (cl:format cl:nil "#An vector of odometry of moving obstacles~%Header header~%int32 number_of_obstacles~%nav_msgs/Odometry[] odom~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OdometryMovingObstacles)))
  "Returns full string definition for message of type 'OdometryMovingObstacles"
  (cl:format cl:nil "#An vector of odometry of moving obstacles~%Header header~%int32 number_of_obstacles~%nav_msgs/Odometry[] odom~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OdometryMovingObstacles>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'odom) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OdometryMovingObstacles>))
  "Converts a ROS message object to a list"
  (cl:list 'OdometryMovingObstacles
    (cl:cons ':header (header msg))
    (cl:cons ':number_of_obstacles (number_of_obstacles msg))
    (cl:cons ':odom (odom msg))
))
