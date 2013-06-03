/*
*   initial_sender.cpp
*   
*   Autor: Enea Scioni
*
*  Simple node to set robot initial pose
*
***************************************************************/

#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "initial_sender");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);
  geometry_msgs::PoseWithCovarianceStamped p;
  double theta_pose;
  
  n.param("x_pose",p.pose.pose.position.x,8.0);
  n.param("y_pose",p.pose.pose.position.y,31.0);
  n.param("z_pose",p.pose.pose.position.z,0.0);
  n.param("theta_pose",theta_pose,0.0);
  
  
  ros::Publisher initial_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2);

  p.header.frame_id = "/map";
  p.header.stamp=ros::Time::now();
  p.pose.pose.orientation=tf::createQuaternionMsgFromYaw(theta_pose);

     

  while(ros::ok())
  {
    initial_pose.publish(p);
    ros::spinOnce();
    loop_rate.sleep();  
  }
}
