//Charanpreet Singh Parmar
//csp6@sfu.ca
//2011-8-19

#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
void PCL_callback(const sensor_msgs::PointCloud msg);
int main(int argc, char **argv)
{
  ros::init(argc, argv, "PCLTEST");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  ros::Subscriber VelSub = n.subscribe("/swissranger/pointcloud_raw", 1000, PCL_callback);

  ros::spin();
  return 0;
}
void PCL_callback(const sensor_msgs::PointCloud msg)
{
  sensor_msgs::PointCloud2 Test;
  sensor_msgs::ChannelFloat32 Channel;
  for (unsigned int i = 0 ; i < cloud1_.channels.size() ; ++i)
    for (unsigned int j = 0 ; j < cloud2_.channels.size() ; ++j)
      if (cloud1_.channels[i].name == cloud2_.channels[j].name)
      {
        ROS_ASSERT(cloud1_.channels[i].values.size() == cloud1_.points.size());
        ROS_ASSERT(cloud2_.channels[j].values.size() == cloud2_.points.size());
        unsigned int oc = out.channels.size();
        out.channels.resize(oc + 1);
        out.channels[oc].name = cloud1_.channels[i].name;
        out.channels[oc].values.resize(cloud1_.channels[i].values.size() + cloud2_.channels[j].values.size());
        std::copy(cloud1_.channels[i].values.begin(), cloud1_.channels[i].values.end(), out.channels[oc].values.begin());
        std::copy(cloud2_.channels[j].values.begin(), cloud2_.channels[j].values.end(), out.channels[oc].values.begin() + cloud1_.channels[i].values.size());
        break;
      }

}
