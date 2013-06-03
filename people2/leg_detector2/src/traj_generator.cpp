#include "ros/ros.h"
#include "people_msgs/PositionMeasurement.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <visualization_msgs/Marker.h>

class PeoplePositionVelocity
{
public:
string object_id;
people_msgs::PositionMeasurement position;
float v[3];
  SavedFeature(const people_msgs::PositionMeasurement::ConstPtr& people_meas)
  {
   position=*people_meas;
  }

}; 

class TrajGenerator
{

public:
NodeHandle nh_;
TransformListener tfl_;

list<SavedFeature*> saved_features_;
list<PeoplePositionVelocity*> people_position_velocity_;

message_filters::Subscriber<people_msgs::PositionMeasurement> people_sub_;
tf::MessageFilter<people_msgs::PositionMeasurement> people_notifier_;

TrajGenerator(ros::NodeHandle nh) :
    nh_(nh), 
people_sub_(nh_,"people_tracker_filter",10),
people_notifier_(people_sub_,tfl_,fixed_frame,10)
{
people_notifier_.registerCallback(boost::bind(&LegDetector::peopleCallback, this, _1));
      people_notifier_.setTolerance(ros::Duration(0.01));

markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
}

~TrajGenerator()
{
}

void peopleCallback(const people_msgs::PositionMeasurement::ConstPtr& people_meas)
{
list<PeoplePositionVelocity*>::iterator begin = people_position_velocity_.begin();
list<PeoplePositionVelocity*>::iterator end = people_position_velocity_.end();
list<PeoplePositionVelocity*>::iterator it1, it2;
bool found=0;
    for (it1=begin;it1!=end;it1++)
    {
       if((*it1)->object_id == people_meas->object_id)
       {
        found=1;
        //if time stamp differ by 1s
        if (people_meas->header.stamp - (*it1)->position->header.stamp > ros::Duration().fromSec(1))
        {
	//trasform might be needed
	 v[0]=people_meas->pos[0]-(*it1)->position->pos[0];
	 v[1]=people_meas->pos[1]-(*it1)->position->pos[1];
	 v[2]=0;
	 it1->position=*people_meas;
	//publish velocity marker
            visualization_msgs::Marker m;
            m.header.stamp = (*it1)->position->header.stamp;
            m.header.frame_id = fixed_frame;
            m.ns = "VELOCITY";
            m.id = i;
            m.type = m.ARROW;
            m.pose.position.x = (*it1)->position->pos[0];
            m.pose.position.y = (*it1)->position->pos[1];
            m.pose.position.z = 0;
            m.scale.x = .2;
            m.scale.y = .2;
            m.scale.z = .2;
            m.color.a = 1;
            m.color.g = 1;
            m.lifetime = ros::Duration(0.5);
            markers_pub_.publish(m);	 
	}
        break;
       }
    }
    if (!found)
    {
      list<PeoplePositionVelocity*>::iterator new_saved = people_position_velocity_.insert(people_position_velocity_.end(), new PeoplePositionVelocity(people_meas));
    }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"traj_generator");
  g_argc = argc;
  g_argv = argv;
  ros::NodeHandle nh;
  TrajGenerator tg(nh);
  ros::spin();
  
  return 0;
}
