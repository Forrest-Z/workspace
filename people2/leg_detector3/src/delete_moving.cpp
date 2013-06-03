#include "ros/ros.h"
#include <leg_detector3/obstacles_pos_vel.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>
using namespace std;
using namespace ros;
using namespace tf;
static string fixed_frame = "odom_combined";

class Delete_obstacles
{
public:
nav_msgs::OccupancyGrid map;
NodeHandle nh_;
TransformListener tfl_;
ros::Publisher map_;
message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
message_filters::Subscriber<leg_detector3::obstacles_pos_vel> obs_pos_sub_;
tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
tf::MessageFilter<leg_detector3::obstacles_pos_vel> obs_pos_notifier_;

Delete_obstacles(ros::NodeHandle nh) :
    nh_(nh), 
    laser_sub_(nh_,"base_scan",10),
    obs_pos_sub_(nh,"people_position_vel",10),
    laser_notifier_(laser_sub_,tfl_,fixed_frame,10),
    obs_pos_notifier_(obs_pos_sub_,tfl_,fixed_frame,10)
{
map.data = new char[1000000];
map.header.frame_id=fixed_frame;
map.header.stamp=ros::Time::now();
map.info.map_load_time=ros::Time::now();
map.info.resolution=.05;
map.info.width=1000;//cells
map.info.height=1000;//cells
map.info.origin.position.x=0;
map.info.origin.position.y=0;
map.info.origin.position.z=0;
map.info.origin.orientation.x=0;
map.info.origin.orientation.y=0;
map.info.origin.orientation.z=0;
map.info.origin.orientation.w=0;
map_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_modified",0);
laser_notifier_.registerCallback(boost::bind(&Delete_obstacles::laserCallback, this, _1));
obs_pos_notifier_.registerCallback(boost::bind(&Delete_obstacles::delObsCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    obs_pos_notifier_.setTolerance(ros::Duration(0.01));
}
~Delete_obstacles()
{}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
/*
	sensor_msgs::PointCloud cloud;
	laser_geometry::LaserProjection projector_;
	geometry_msgs::PointStamped global_point,laser_point;
	int numofpoints=180/.5;	
	laser_point.header.frame_id="lms_link";	
	laser_point.header.stamp=ros::Time();
	//convert to cartesian
	projector_.projectLaser(*scan,cloud);
	//transforming each point to fixed frame and saving in map
	for (int i=0;i<numofpoints;i++)
	{
		     laser_point.point.x=cloud.points[i].x;
		     laser_point.point.y=cloud.points[i].y;
		     laser_point.point.z=0;
		     try {
    		      tfl_.transformPoint(fixed_frame, laser_point, global_point);
                     } catch(...) {
                     ROS_WARN("TF exception");
		     }		 
		     map.data[global_point.point.x+global_point.point.y*map.info.width]=1;	
	}


*/
    }
void delObsCallback(const leg_detector3::obstacles_pos_vel::ConstPtr& obs)
    {
	int count=0,obs_x=0,obs_y=0;
        count=obs->number_of_obstacles;
	for (int i=0; i<count;i++)
	{
		obs_x=obs->pos_vel[i].pos_x;
		obs_y=obs->pos_vel[i].pos_y;
		for (int x=obs_x-10;x<obs_x+10;x++)
		{
			for(int y=obs_y-10;y<obs_y+10;y++)
			{
			map.data[x+y*map.info.width]=0;
			}
		}
	}
	map.header.stamp=ros::Time::now();
	map_.publish(map);
    } 
};






int main(int argc, char **argv)
{  
  ros::init(argc, argv,"delete_moving_obs");
  ros::NodeHandle nh;
  Delete_obstacles del_obs(nh);
  ros::spin();
  return 0;
}
