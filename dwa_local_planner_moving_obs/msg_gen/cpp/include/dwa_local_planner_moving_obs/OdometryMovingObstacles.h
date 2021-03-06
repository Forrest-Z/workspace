/* Auto-generated by genmsg_cpp for file /home/sid/ros_workspace/dwa_local_planner_moving_obs/msg/OdometryMovingObstacles.msg */
#ifndef DWA_LOCAL_PLANNER_MOVING_OBS_MESSAGE_ODOMETRYMOVINGOBSTACLES_H
#define DWA_LOCAL_PLANNER_MOVING_OBS_MESSAGE_ODOMETRYMOVINGOBSTACLES_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"

namespace dwa_local_planner_moving_obs
{
template <class ContainerAllocator>
struct OdometryMovingObstacles_ {
  typedef OdometryMovingObstacles_<ContainerAllocator> Type;

  OdometryMovingObstacles_()
  : header()
  , number_of_obstacles(0)
  , odom()
  {
  }

  OdometryMovingObstacles_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , number_of_obstacles(0)
  , odom(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _number_of_obstacles_type;
  int32_t number_of_obstacles;

  typedef std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other >  _odom_type;
  std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other >  odom;


  ROS_DEPRECATED uint32_t get_odom_size() const { return (uint32_t)odom.size(); }
  ROS_DEPRECATED void set_odom_size(uint32_t size) { odom.resize((size_t)size); }
  ROS_DEPRECATED void get_odom_vec(std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other > & vec) const { vec = this->odom; }
  ROS_DEPRECATED void set_odom_vec(const std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other > & vec) { this->odom = vec; }
private:
  static const char* __s_getDataType_() { return "dwa_local_planner_moving_obs/OdometryMovingObstacles"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "b75dd1fa87c85e47ac20e03fdf58c4da"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "#An vector of odometry of moving obstacles\n\
Header header\n\
int32 number_of_obstacles\n\
nav_msgs/Odometry[] odom\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertianty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, number_of_obstacles);
    ros::serialization::serialize(stream, odom);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, number_of_obstacles);
    ros::serialization::deserialize(stream, odom);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(number_of_obstacles);
    size += ros::serialization::serializationLength(odom);
    return size;
  }

  typedef boost::shared_ptr< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct OdometryMovingObstacles
typedef  ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<std::allocator<void> > OdometryMovingObstacles;

typedef boost::shared_ptr< ::dwa_local_planner_moving_obs::OdometryMovingObstacles> OdometryMovingObstaclesPtr;
typedef boost::shared_ptr< ::dwa_local_planner_moving_obs::OdometryMovingObstacles const> OdometryMovingObstaclesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace dwa_local_planner_moving_obs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b75dd1fa87c85e47ac20e03fdf58c4da";
  }

  static const char* value(const  ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb75dd1fa87c85e47ULL;
  static const uint64_t static_value2 = 0xac20e03fdf58c4daULL;
};

template<class ContainerAllocator>
struct DataType< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dwa_local_planner_moving_obs/OdometryMovingObstacles";
  }

  static const char* value(const  ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#An vector of odometry of moving obstacles\n\
Header header\n\
int32 number_of_obstacles\n\
nav_msgs/Odometry[] odom\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertianty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.number_of_obstacles);
    stream.next(m.odom);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct OdometryMovingObstacles_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::dwa_local_planner_moving_obs::OdometryMovingObstacles_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "number_of_obstacles: ";
    Printer<int32_t>::stream(s, indent + "  ", v.number_of_obstacles);
    s << indent << "odom[]" << std::endl;
    for (size_t i = 0; i < v.odom.size(); ++i)
    {
      s << indent << "  odom[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::nav_msgs::Odometry_<ContainerAllocator> >::stream(s, indent + "    ", v.odom[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // DWA_LOCAL_PLANNER_MOVING_OBS_MESSAGE_ODOMETRYMOVINGOBSTACLES_H

