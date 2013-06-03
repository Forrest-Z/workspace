#include <ros/ros.h>
#include <MTTD_msgs/People.h>
#include <sensor_msgs/PointCloud.h>

class PeopleStamp
{
  public:
      PeopleStamp()
      {
	read = _n.subscribe( "/MTTDO1/peopledata", 1, (boost::function < void(const MTTD_msgs::People::ConstPtr&)>) boost::bind( &PeopleStamp::visualize, this, _1 ));
	vis_pub = _n.advertise<sensor_msgs::PointCloud>( "nuvola", 0 );

      }
      
      ~PeopleStamp(){};
      
      void visualize(const MTTD_msgs::PeopleConstPtr& msg)
      {
	ROS_INFO("received");
 	sensor_msgs::PointCloud cloud;
 	cloud.header.frame_id = "map";
 	cloud.header.stamp = ros::Time::now();
 	cloud.points.resize(msg->num);
	
	for(int i=0;i<msg->num;i++)
	{
	  cloud.points[i].x=msg->people[i].pos.x;
	   cloud.points[i].y=msg->people[i].pos.y;
	    cloud.points[i].z=msg->people[i].pos.z;
	}
 	//cloud.points = msg->coordinates;
 	
 	vis_pub.publish( cloud );
      }
      
  private:
    ros::NodeHandle _n;
    ros::Subscriber read;
    ros::Publisher vis_pub;
};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"ros_reader");
  
  PeopleStamp stamp;
  ros::spin();
  
  return 0;
}
