#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Point.h"
#include <math.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <dwa_local_planner_moving_obs/OdometryMovingObstacles.h>
using namespace std;
using namespace ros;
using namespace tf;
#define ODOM "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"
const int numberOfPeople=2;
// scale for attractive force towards goal(vel direction)
const double goalScale=15;
// min dist for repulsion force to act
const double dmin=3;
const double dminpeople=.95;
// scale for repultion force
const double obsScale=.8;
const double dT=.1;
const double corridor_width=.45;
ofstream outfile;

class OdometryMovingObs
{
public:
std::vector<ros::Subscriber> odom_subs_;
dwa_local_planner_moving_obs::OdometryMovingObstacles obstacle_pos_vel_;
std::vector<ros::Publisher> cmd_vel_pubs_;
std::vector<geometry_msgs::Twist> initialVelocities;
std::vector<geometry_msgs::Twist> obstacleVelocities;
std::vector<float> flags;
nav_msgs::Odometry robotOdometry_;
ros::Publisher odometry_pub_;
ros::NodeHandle n_;
ros::Publisher markers_pub_;

OdometryMovingObs(ros::NodeHandle nh);
void odomReceived(int idx, const boost::shared_ptr<nav_msgs::Odometry const>& msg);
const char * mapName(const char *name, size_t robotID);
void initialize_velocities();
void updateVelocities(int);
};

void OdometryMovingObs::initialize_velocities(){
  geometry_msgs::Twist vel;
  obstacle_pos_vel_.number_of_obstacles=numberOfPeople;
  vel.linear.x=-0.2;vel.linear.y=0;vel.linear.z=0;
   vel.angular.x=0;vel.angular.y=0;vel.angular.z=0;
   obstacleVelocities.push_back(vel);
   initialVelocities.push_back(vel);
   flags.push_back(1);
  vel.linear.x=0.3;vel.linear.y=0;vel.linear.z=0;
   obstacleVelocities.push_back(vel);
   initialVelocities.push_back(vel);
   flags.push_back(1);
}

void OdometryMovingObs::updateVelocities(int idx){
 if (flags[idx-1]<.5) return;
 double Fx,Fy,dx,dy,dist,R,Rx,Ry,acc_x,acc_y;
 Fx=goalScale*(initialVelocities[idx-1].linear.x-obstacleVelocities[idx-1].linear.x);
 Fy=goalScale*(initialVelocities[idx-1].linear.y-obstacleVelocities[idx-1].linear.y);
 //Repulsion due to robot
 dx=(robotOdometry_.pose.pose.position.x-obstacle_pos_vel_.odom[idx-1].pose.pose.position.x);
 dy=(robotOdometry_.pose.pose.position.y-obstacle_pos_vel_.odom[idx-1].pose.pose.position.y);
 dist = sqrt(dx*dx+dy*dy);
 if (dist<dmin){
   R= obsScale*(1/(dist-.6) - 1/(dmin));
   Rx=R*dx;
   Ry=R*dy;
   }
 else{
   Rx=0;
   Ry=0;
 }
 //Repulsion due to other people
 for (int j=0;j<obstacleVelocities.size();j++){
   if (idx-1==j) continue;
   dx=(obstacle_pos_vel_.odom[j].pose.pose.position.x-obstacle_pos_vel_.odom[idx-1].pose.pose.position.x);
   dy=(obstacle_pos_vel_.odom[j].pose.pose.position.y-obstacle_pos_vel_.odom[idx-1].pose.pose.position.y);
   dist = sqrt(dx*dx+dy*dy);
   if (dist<dmin){
     R= obsScale*(1/(dist-.6) - 1/(dminpeople));
     Rx=Rx+R*dx;
     Ry=Ry+R*dy;
     }
 }
 //Net accl
 acc_x = Fx - Rx;
 acc_y = Fy - Ry;

//update velocity
 obstacleVelocities[idx-1].linear.x = obstacleVelocities[idx-1].linear.x + acc_x*dT; 
 obstacleVelocities[idx-1].linear.y = obstacleVelocities[idx-1].linear.y + acc_y*dT;

//velocity can't be negetive
 if (obstacleVelocities[idx-1].linear.x*initialVelocities[idx-1].linear.x<0) obstacleVelocities[idx-1].linear.x=0;
 if (obstacleVelocities[idx-1].linear.y*initialVelocities[idx-1].linear.y<0) obstacleVelocities[idx-1].linear.y=0;

 //if obstacle is hitting the wall make vel =0 in that direction
 if (obstacle_pos_vel_.odom[idx-1].pose.pose.position.y<-corridor_width || obstacle_pos_vel_.odom[idx-1].pose.pose.position.y>corridor_width)
   obstacleVelocities[idx-1].linear.y=0;
}

OdometryMovingObs::OdometryMovingObs(ros::NodeHandle nh):
 n_(nh)
{
 OdometryMovingObs::initialize_velocities();
 for (int r=0;r<=numberOfPeople;r++){
    odom_subs_.push_back(n_.subscribe<nav_msgs::Odometry>(mapName(ODOM,r), 10, boost::bind(&OdometryMovingObs::odomReceived, this, r, _1)));
    }
 for (int r=1;r<=numberOfPeople;r++){
    cmd_vel_pubs_.push_back(n_.advertise<geometry_msgs::Twist>(mapName(CMD_VEL,r), 10));
    }
 odometry_pub_= n_.advertise<dwa_local_planner_moving_obs::OdometryMovingObstacles>("OdometryMovingObstacles",10);
 markers_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
}

const char * OdometryMovingObs::mapName(const char *name, size_t robotID)
{
  static char buf[100];
  snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
  return buf;
}


void OdometryMovingObs::odomReceived(int idx, const boost::shared_ptr<nav_msgs::Odometry const>& msg)
{
// store odometry data for robot
 if (idx==0){
   robotOdometry_=*msg;
   return;
   }
// store odometry data for moving obstacle
 if(obstacle_pos_vel_.odom.size()+1>idx)
   obstacle_pos_vel_.odom[idx-1]=(*msg);
 else
   obstacle_pos_vel_.odom.push_back(*msg);
outfile<<idx<<' '<<(obstacle_pos_vel_.odom[idx-1]).pose.pose.position.x<<' '<<(obstacle_pos_vel_.odom[idx-1]).pose.pose.position.y<<' '<<(obstacle_pos_vel_.odom[idx-1]).twist.twist.linear.x<<'\n';

//update velocities
   updateVelocities(idx);

//Adding vel and flag info to Odometry msg (Stage does not seem to publish this) 
   obstacle_pos_vel_.odom[idx-1].twist.twist.linear.x=obstacleVelocities[idx-1].linear.x;
   obstacle_pos_vel_.odom[idx-1].twist.twist.linear.y=obstacleVelocities[idx-1].linear.y;
//   obstacle_pos_vel_.odom[idx-1].header.seq=int(flags[idx-1]);
   obstacle_pos_vel_.odom[idx-1].header.seq=0;
//publish new_vel for StageROS (converting axis x=y and y=-x before publishing to stage)
   geometry_msgs::Twist vel;
   vel.linear.x=-obstacleVelocities[idx-1].linear.y;vel.linear.y=obstacleVelocities[idx-1].linear.x;vel.linear.z=0;
   vel.angular.x=0;vel.angular.y=0;vel.angular.z=0;
   cmd_vel_pubs_[idx-1].publish(vel);

//publish visualization marker 
visualization_msgs::Marker m;
    m.header.stamp = msg->header.stamp;
    m.header.frame_id = "/map";
    m.ns = "Obstacles";
    m.id = idx;
    m.type = m.CYLINDER;
    m.pose.position.x = obstacle_pos_vel_.odom[idx-1].pose.pose.position.x;
    m.pose.position.y = obstacle_pos_vel_.odom[idx-1].pose.pose.position.y;
    m.pose.position.z = 0.5;
    m.scale.x = .6;
    m.scale.y = .6;
    if (flags[idx-1]<.5){
      m.scale.z = 1; m.color.a = 1; m.color.g = 0;
      }
    else{
      m.scale.z = 1; m.color.a = 1; m.color.g = 1;
      }
    m.lifetime = ros::Duration(10000);
    markers_pub_.publish(m);   

//publish all moving obstacles' odometry for move_base
if (idx==1) {
  obstacle_pos_vel_.header.seq=corridor_width;
  odometry_pub_.publish(obstacle_pos_vel_);
 } 
}

int main(int argc, char **argv)
{  
  outfile.open("/home/sid/testoutput.txt");
  ros::init(argc, argv,"main_test");
  ros::NodeHandle nh;
  OdometryMovingObs tg(nh);
  ros::spin();
  outfile.close();
  return 0;
}
