//Charanpreet Singh Parmar
//csp6@sfu.ca
//2011-8-19

#include "ros/ros.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include <tf/transform_broadcaster.h>

//This is in a seperate node from the Brake node because it is better to have a seperate thread for this so it can publish more often
void CSCallback(const sensor_msgs::JointState msg)
{
  static tf::TransformBroadcaster broadcaster;//Transform Broadcaster
  tf::Transform transform;//Transform info
  float pan, tilt;
  pan = msg.position[0];//assumed that msg.position[0] is Pan
  tilt = -msg.position[1];//assumed that msg.position[1] is Tilt (angle is measured from other end of axis, thus the (-) sign)
  transform.setOrigin( tf::Vector3(0.288, 0.0, 0.725) );//Took measurements to get these values
  transform.setRotation( tf::Quaternion(pan, tilt, 0));//Convert from angles to quaternion
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_footprint", "/PTU_Frame"));//publish TF
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "PTU_PCL");
  ros::NodeHandle n;

  ros::Subscriber CurrentState = n.subscribe("/ptu/state", 1000, CSCallback);

  ros::spin();
  return 0;
}
