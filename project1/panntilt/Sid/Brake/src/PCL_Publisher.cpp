//Charanpreet Singh Parmar
//csp6@sfu.ca
//2011-8-19

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud_conversion.h"

void PCL_callback(const sensor_msgs::PointCloud msg);
void RefreshData(const ros::TimerEvent& event);

ros::Publisher PCL_Pub;
ros::Publisher PCL2_Pub;

double period;
sensor_msgs::PointCloud PCL;
sensor_msgs::PointCloud2 PCL2;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PCL_Updater");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  n_private.param("period", period, 0.1);

  ros::Subscriber PCLSub = n.subscribe("/cloud_out", 1000, PCL_callback);
  ros::Timer timer = n.createTimer(ros::Duration(period), RefreshData);

  PCL_Pub = n.advertise<sensor_msgs::PointCloud>("/Assembled_PCL", 10);//Publish PointCloud
  PCL2_Pub = n.advertise<sensor_msgs::PointCloud2>("/Assembled_PCL2", 10);//Publish PointCloud2

  ros::spin();
  return 0;
}
void PCL_callback(const sensor_msgs::PointCloud msg)
{
  PCL_Pub.publish(msg);//Copying can take a while so it's best to publish the message before copying
  PCL = msg;//Copy Received PointCloud
  sensor_msgs::convertPointCloudToPointCloud2 (PCL, PCL2);//Transform into PointCloud2
}
void RefreshData(const ros::TimerEvent& event)
{
  //Update Header Times to now
  PCL.header.stamp = ros::Time::now();
  PCL2.header.stamp = ros::Time::now();

  //Publish PointCloud and PointCloud2
  PCL_Pub.publish(PCL);
  PCL2_Pub.publish(PCL2);
}
