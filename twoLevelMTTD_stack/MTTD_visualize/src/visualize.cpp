#include <ros/ros.h>
#include <MTTD_msgs/People.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>
#include <ctime>
#include <vector>
#include <sstream>

#define MAX_ID_NUM_BUFF 6

class PeopleStamp
{
  public:
      PeopleStamp()
      {
	_read = _n.subscribe( "/MTTDO1/peopledata", 1, (boost::function < void(const MTTD_msgs::People::ConstPtr&)>) boost::bind( &PeopleStamp::visualize, this, _1 ));
	_vis_pub = _n.advertise<visualization_msgs::Marker>("targets", 5);
	
	//Init marker
	marker.header.frame_id = "/map";
 	marker.header.stamp = ros::Time::now();
 	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1;
	marker.scale.x = 0.3; //Point dimension
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	
	//Create table colors TODO: parametrize target limit (20)
	//srand ( time(NULL) );
	for(unsigned int i=0;i<20;i++)
	{
	  color.a=1;
	  color.g=(rand() % 100)*0.01;//0;
	  color.r=(rand() % 100)*0.01;//1;
	  color.b=(rand() % 100)*0.01;//0
	  colors.push_back(color);
	}
	
	//Init text marker
	text.header.frame_id = "/map";
	text.header.stamp = ros::Time::now();
	text.ns = "my_namespace";
	text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text.action = visualization_msgs::Marker::ADD;
	text.pose.position.z = 0.5;
	text.pose.orientation.x = 0.0;
	text.pose.orientation.y = 0.0;
	text.pose.orientation.z = 0.0;
	text.pose.orientation.w = 1;
	text.scale.x = 0.5; //Text dimension
	text.scale.y = 0.5;
	text.scale.z = 0.5;
	text.color.a = 1.0;
	text.color.r = 0.0;
	text.color.g = 1.0;
	text.color.b = 0.0;

      }
      
      ~PeopleStamp(){};
      
      void visualize(const MTTD_msgs::PeopleConstPtr& msg)
      {
	ROS_DEBUG("received people data to visualize");

	//srand ( time(NULL) );
	marker.points.clear();
	marker.colors.clear();
	text.colors.clear();
	for(int i=0;i<msg->num;i++)
	{
	  point.x=msg->people[i].pos.x;
	  point.y=msg->people[i].pos.y;
	  point.z=msg->people[i].pos.z;
	  marker.points.push_back(point);
	  marker.colors.push_back(colors[i]);
	}
 	
 	
 	_vis_pub.publish( marker );
	
	//visualize id number as text marker 
	//TODO::add bool parameter yes/no id number to visualize
	for(int i=0;i<msg->num;i++)
	{
	  text.id=i+1;
	  text.pose.position.x=msg->people[i].pos.x;
	  text.pose.position.y=msg->people[i].pos.y;
	  text.color=colors[i];
	  text.text=msg->people[i].object_id;
	  _vis_pub.publish(text);
	}
      }
      
  private:
    ros::NodeHandle _n;
    ros::Subscriber _read;
    ros::Publisher _vis_pub;
    char _buffer[MAX_ID_NUM_BUFF];
  protected:
        visualization_msgs::Marker marker;
	visualization_msgs::Marker text;
	std::ostringstream aux_text;
	geometry_msgs::Point point;
	std_msgs::ColorRGBA color;
	std::vector<std_msgs::ColorRGBA> colors;
	 
};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"ros_reader");
  
  PeopleStamp stamp;
  ros::spin();
  
  return 0;
}
