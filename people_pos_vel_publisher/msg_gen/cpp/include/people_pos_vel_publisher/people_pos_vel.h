/* Auto-generated by genmsg_cpp for file /home/sid/ros_workspace/people_pos_vel_publisher/msg/people_pos_vel.msg */
#ifndef PEOPLE_POS_VEL_PUBLISHER_MESSAGE_PEOPLE_POS_VEL_H
#define PEOPLE_POS_VEL_PUBLISHER_MESSAGE_PEOPLE_POS_VEL_H
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
#include "people_pos_vel_publisher/pos_vel.h"

namespace people_pos_vel_publisher
{
template <class ContainerAllocator>
struct people_pos_vel_ {
  typedef people_pos_vel_<ContainerAllocator> Type;

  people_pos_vel_()
  : header()
  , number_of_obstacles(0)
  , pos_vel()
  {
  }

  people_pos_vel_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , number_of_obstacles(0)
  , pos_vel()
  {
    pos_vel.assign( ::people_pos_vel_publisher::pos_vel_<ContainerAllocator> (_alloc));
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _number_of_obstacles_type;
  int32_t number_of_obstacles;

  typedef boost::array< ::people_pos_vel_publisher::pos_vel_<ContainerAllocator> , 20>  _pos_vel_type;
  boost::array< ::people_pos_vel_publisher::pos_vel_<ContainerAllocator> , 20>  pos_vel;


  ROS_DEPRECATED uint32_t get_pos_vel_size() const { return (uint32_t)pos_vel.size(); }
private:
  static const char* __s_getDataType_() { return "people_pos_vel_publisher/people_pos_vel"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "010f76888b4a97e54e8975ad011367ee"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "#An array of position and velocities of moving obstacles\n\
Header header\n\
int32 number_of_obstacles\n\
people_pos_vel_publisher/pos_vel[20] pos_vel\n\
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
MSG: people_pos_vel_publisher/pos_vel\n\
# A representation of position and velocity of object\n\
Header header\n\
float32 pos_x\n\
float32 pos_y\n\
float32 pos_z\n\
float32 vel_x\n\
float32 vel_y\n\
float32 vel_z\n\
bool flag\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, number_of_obstacles);
    ros::serialization::serialize(stream, pos_vel);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, number_of_obstacles);
    ros::serialization::deserialize(stream, pos_vel);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(number_of_obstacles);
    size += ros::serialization::serializationLength(pos_vel);
    return size;
  }

  typedef boost::shared_ptr< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct people_pos_vel
typedef  ::people_pos_vel_publisher::people_pos_vel_<std::allocator<void> > people_pos_vel;

typedef boost::shared_ptr< ::people_pos_vel_publisher::people_pos_vel> people_pos_velPtr;
typedef boost::shared_ptr< ::people_pos_vel_publisher::people_pos_vel const> people_pos_velConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace people_pos_vel_publisher

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> > {
  static const char* value() 
  {
    return "010f76888b4a97e54e8975ad011367ee";
  }

  static const char* value(const  ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x010f76888b4a97e5ULL;
  static const uint64_t static_value2 = 0x4e8975ad011367eeULL;
};

template<class ContainerAllocator>
struct DataType< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> > {
  static const char* value() 
  {
    return "people_pos_vel_publisher/people_pos_vel";
  }

  static const char* value(const  ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#An array of position and velocities of moving obstacles\n\
Header header\n\
int32 number_of_obstacles\n\
people_pos_vel_publisher/pos_vel[20] pos_vel\n\
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
MSG: people_pos_vel_publisher/pos_vel\n\
# A representation of position and velocity of object\n\
Header header\n\
float32 pos_x\n\
float32 pos_y\n\
float32 pos_z\n\
float32 vel_x\n\
float32 vel_y\n\
float32 vel_z\n\
bool flag\n\
\n\
";
  }

  static const char* value(const  ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.number_of_obstacles);
    stream.next(m.pos_vel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct people_pos_vel_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::people_pos_vel_publisher::people_pos_vel_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "number_of_obstacles: ";
    Printer<int32_t>::stream(s, indent + "  ", v.number_of_obstacles);
    s << indent << "pos_vel[]" << std::endl;
    for (size_t i = 0; i < v.pos_vel.size(); ++i)
    {
      s << indent << "  pos_vel[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::people_pos_vel_publisher::pos_vel_<ContainerAllocator> >::stream(s, indent + "    ", v.pos_vel[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // PEOPLE_POS_VEL_PUBLISHER_MESSAGE_PEOPLE_POS_VEL_H

