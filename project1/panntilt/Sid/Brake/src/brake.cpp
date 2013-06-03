//Charanpreet Singh Parmar
//csp6@sfu.ca
//2011-8-19

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ros/message.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"

//PTUCLient
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "ptu_control/PtuGotoAction.h"
#include "ptu_control/PtuResetAction.h"

//TF for PTU
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/JointState.h"

//For Assembler
#include <laser_assembler/AssembleScans.h>//For Assembler/Snapshotter Client
#include <sensor_msgs/PointCloud.h>//For PointCloud Publisher


ros::Publisher pub_vel;//Velocity Publisher/Forwarder
ros::Publisher pub_pcl;//PointCloud Publisher
ros::ServiceClient client;//Point Cloud Assembler/Snapshotter Client

void pose_callback(const nav_msgs::Odometry::ConstPtr& msg);//Calback for Pose
void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);//Callback for Velocity
void roatatePTU();//Function to Rotate the PTU

bool Init = false;//Bool used to capture Pose to startup
bool stop = false;//Bool used to toggle brakes
int LeftRight = 1, UpDown = 1, Timer_sec;//Int used to toggle between Left-to-Right and Right-to-Left rotations, as well as for Up-to-Down and Down-to-Up rotations
nav_msgs::Odometry Last_Pose;//Last Scanned Odometry Information
double dist_tolerance, theta_tolerance;//Allowed change in Distance and Angle before Re-Scan
double delta_pan, pan_range, delta_tilt, tilt_range, pan_offset, tilt_offset;//The Change in Angle for panning, the total range/tilt that is panned, and centre to pan/tilt around
ros::Time Last, Timer;

typedef actionlib::SimpleActionClient<ptu_control::PtuGotoAction> Client;//Client type for the PTU

class PTU
{
public:
  PTU() : SetPTU("SetPTUState", true)
  {
    SetPTU.waitForServer();//Wait for SetPTUState to be active
  }
  void pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if(Init)
    {
      //Capture current Pose
      double x, y, z, theta, thetax, thetay, thetaz, thetaw, theta1, theta2, dist;//Intermediate Variables
      x=msg->pose.pose.position.x - Last_Pose.pose.pose.position.x;//Change in x
      y=msg->pose.pose.position.y - Last_Pose.pose.pose.position.y;//Change in y
      z=msg->pose.pose.position.z - Last_Pose.pose.pose.position.z;//Change in z
      thetax=msg->pose.pose.orientation.x;//Current Quaterion x
      thetay=msg->pose.pose.orientation.y;//Current Quaterion y
      thetaz=msg->pose.pose.orientation.z;//Current Quaterion z
      thetaw=msg->pose.pose.orientation.w;//Current Quaterion w
      //Convert from Quaternion to Degree rotation around Z-Axis. Need to convert first and CANNOT simply
      //subtract old x,y,z,w values from new ones, as in the case for the x,y,z for the position since the
      //Quaternion coordinate system does not work that way. MUST convert to degrees before looking for change
      //in angle.
      theta1=atan2(2*(Last_Pose.pose.pose.orientation.w*Last_Pose.pose.pose.orientation.z + Last_Pose.pose.pose.orientation.x*Last_Pose.pose.pose.orientation.y),(1 - 2*(Last_Pose.pose.pose.orientation.z*Last_Pose.pose.pose.orientation.z + Last_Pose.pose.pose.orientation.y*Last_Pose.pose.pose.orientation.y)));//Last angle
      theta2=atan2(2*(thetaw*thetaz + thetax*thetay),(1 - 2*(thetaz*thetaz + thetay*thetay)));//Current Angle
      theta = theta1-theta2;//Change in Angle
      dist= sqrt(x*x + y*y + z*z);//Change in distance
      if((dist >= dist_tolerance) || ( fabs(theta) >= theta_tolerance))//Check if either Angle or Distance has changed enough for a Re-Scan
      {
        stop = true;//Set the Powerbot to Stop
        //Save Current Pose
        Last_Pose.pose.pose.position.x=msg->pose.pose.position.x;
        Last_Pose.pose.pose.position.y=msg->pose.pose.position.y;
        Last_Pose.pose.pose.position.z=msg->pose.pose.position.z;
        Last_Pose.pose.pose.orientation.x=msg->pose.pose.orientation.x;
        Last_Pose.pose.pose.orientation.y=msg->pose.pose.orientation.y;
        Last_Pose.pose.pose.orientation.z=msg->pose.pose.orientation.z;
        Last_Pose.pose.pose.orientation.w=msg->pose.pose.orientation.w;
      }
    }
    else
    {
      //Capture initial pose on Node Start-Up
      Init = true;
      stop = true;
      Last_Pose.pose.pose.position.x=msg->pose.pose.position.x;
      Last_Pose.pose.pose.position.y=msg->pose.pose.position.y;
      Last_Pose.pose.pose.position.z=msg->pose.pose.position.z;
      Last_Pose.pose.pose.orientation.x=msg->pose.pose.orientation.x;
      Last_Pose.pose.pose.orientation.y=msg->pose.pose.orientation.y;
      Last_Pose.pose.pose.orientation.z=msg->pose.pose.orientation.z;
      Last_Pose.pose.pose.orientation.w=msg->pose.pose.orientation.w;
    }
    if(ros::Time::now().sec - Last.sec >= Timer.sec)
    {
      stop = true;
      ROS_INFO("%d", Timer.sec);
    }
  }
  void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
  {

    if(stop)//If Pose Callback sets stop to true, set velocities to stop
    {
      geometry_msgs::Twist Vel;//Velocity Structure for setting user defined Velocity
      Vel.angular.z = 0;
      Vel.linear.x = 0;
      pub_vel.publish(Vel);//STOP
      ROS_DEBUG("Stop");
      roatatePTU();
      stop = false;//GO (Next time this is called)
      ROS_DEBUG("Go");
    }
    else//Otherwise pass through current speed
      pub_vel.publish(msg);
  }
  void roatatePTU()
  {
    laser_assembler::AssembleScans srv;//Assemble Scans Service type
    sensor_msgs::PointCloud cloud_out;//PointCloud to be outputted

    cloud_out.points.clear();//Clear Old Points
    cloud_out.channels.clear();//Clear old Channels
    cloud_out.channels.resize(2);//Resize Channels
    cloud_out.channels[0].name = "intensity";//Set Channel 0 to be intensity
    cloud_out.channels[1].name =  "confidence";//Set Channel 1 to be confidence
    Last = ros::Time::now();//Set Initial Time

    for(int count_pan = 0; count_pan <= pan_range/delta_pan; count_pan++)//For panning the PTU/Swissranger
    {
      for(int count_tilt =0; count_tilt <= tilt_range/delta_tilt; count_tilt++)//For Tilting the PTU/Swissranger
      {
        //Go to or stay at one of the sides
        //IMPORTANT: Although all of ROS is in Radians, the PTU46 uses units of DEGREES so be careful when using the PTU parameters and remember to convert first!!
        if(tilt_range/delta_tilt != 0)
          Goal.tilt = (float) UpDown*(tilt_range/2 - delta_tilt*count_tilt) + tilt_offset;
        else
          Goal.tilt = tilt_offset;
        Goal.pan_vel = 30;//Deg/s
        Goal.tilt_vel = 30;//Deg/s
        if(pan_range/delta_pan != 0)
          Goal.pan = (float) LeftRight*(pan_range/2 - delta_pan*count_pan) + pan_offset;
        else
          Goal.pan = pan_offset;
        SetPTU.sendGoalAndWait(Goal);//Pan/Tilt to the desired Postion

        ROS_DEBUG("CHECK Pan: %i, Tilt: %i", count_pan, count_tilt);
        ros::Duration(0.4).sleep();//Sleep while PointCloud settles. Number attained from Experimentation.

        //Assemble the PointCloud from when Last called and now. Since the buffer only holds one cloud, only the latest data will be received
        srv.request.begin = Last;
        srv.request.end = ros::Time::now();

        //Assemble CLouds
        if(client.call(srv))//Call Service
        {
          if (srv.response.cloud.points.size() <= 0)
            break;//If cloud is empty, break
          //Copy points to output PointCloud
          for(unsigned int i=0; i<srv.response.cloud.points.size();i++)
            cloud_out.points.push_back(srv.response.cloud.points[i]);
          //Copy channel data to output Point_cloud
          //Code modified from merge_clouds.cpp in laser_assembler
          for (unsigned int i = 0 ; i < cloud_out.channels.size() ; ++i)
            for (unsigned int j = 0 ; j < srv.response.cloud.channels.size() ; ++j)
              if (cloud_out.channels[i].name == srv.response.cloud.channels[j].name)
              {
                unsigned int cloud_out_size_init = cloud_out.channels[i].values.size();//Amount of data already existing in channel
                cloud_out.channels[i].values.resize(cloud_out.channels[i].values.size() + srv.response.cloud.channels[j].values.size());//Resize channel to fit new Data
                std::copy(srv.response.cloud.channels[j].values.begin(), srv.response.cloud.channels[j].values.end(), cloud_out.channels[i].values.begin() + cloud_out_size_init);//Copy Received data to output PointCloud
                break;//Break when done
              }
        }
        Last = ros::Time::now();//Update Last time for Updating PointCloud
      }
      cloud_out.header.frame_id = srv.response.cloud.header.frame_id;//Make the output PointCloud have same frame_id, should be /map
      cloud_out.header.stamp = ros::Time::now();//Make the time stamp for the Header to be now
      pub_pcl.publish(cloud_out);

      //Toggle between going Up-to-down and Down-to-Right
      if(UpDown == 1)
        UpDown = -1;
      else
        UpDown = 1;
    }
    //Toggle between going Left-to-Right and Right-to-Left
    if(LeftRight == 1)
      LeftRight = -1;
    else
      LeftRight = 1;

    Last = ros::Time::now();
  }

private:
  Client SetPTU;//Used to set Pan-Tilt
  ptu_control::PtuGotoGoal Goal;//Goal to be Set

};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Brake");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  PTU ptu;

  n_private.param("dist_tolerance", dist_tolerance , 2.0);//Default set to 2 metres
  n_private.param("theta_tolerance", theta_tolerance , M_PI/3);//Default set to 60 degrees
  n_private.param("pan", delta_pan , 30.0);//Default set to 30 degrees
  n_private.param("pan_range", pan_range , 180.0);//Default set to 30 degrees
  n_private.param("pan_offset", pan_offset, 0.0);//The centre point to be panned about
  n_private.param("tilt", delta_tilt , 30.0);//Default set to 30 degrees
  n_private.param("tilt_range", tilt_range , 30.0);//Default set to 30 degrees
  n_private.param("tilt_offset", tilt_offset, 0.0);//The centre point to be tilted about
  n_private.param("timer", Timer_sec, 60);//Maximum Time before re-panning
  Timer.sec = (uint32_t) Timer_sec;

  client = n.serviceClient<laser_assembler::AssembleScans>("/AssemblerServer/build_cloud");//AssembleScans Service

  ros::Subscriber CurrentPose = n.subscribe("/pose", 1000, &PTU::pose_callback, &ptu);//Subscribe to current Pose
  ros::Subscriber VelSub = n.subscribe("/cmd_vel", 1000, &PTU::vel_callback, &ptu);//Subscribe to Velocity Commands

  pub_vel = n.advertise<geometry_msgs::Twist>("/p2os/cmd_vel",1000);//Publish Velocity to /p2os/cmd_vel
  pub_pcl = n.advertise<sensor_msgs::PointCloud>("/cloud_out", 1000);//Publish Assembled PointCloud

  Last = ros::Time::now();

  ros::spin();
  return 0;

}
