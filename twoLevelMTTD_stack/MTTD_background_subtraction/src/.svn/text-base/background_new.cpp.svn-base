#include <ros/ros.h>
#include <ros/assert.h>
#include <vector>
#include <map>
//#include "map/map.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include <laser_geometry/laser_geometry.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//Controlla la mappa e i dati del laser e controlla le cose in comune 
 class CheckBackground{
   
   public:
      CheckBackground();
      
      ~CheckBackground()
	{
	}
	

	void dataReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
	{
	    sensor_msgs::PointCloud cloud;
	    laser_geometry::LaserProjection projector;
	 
	    projector.transformLaserScanToPointCloud("map",*laser_scan, cloud,*tf_);
	    
	    sensor_msgs::PointCloud clean_cloud;
	    sensor_msgs::PointCloud temp_cloud;
	    clean_cloud.header.stamp=ros::Time::now();
	    clean_cloud.header.frame_id="map";
	    
	    temp_cloud.points.resize(cloud.points.size());
	    int j=0;
	    for(unsigned i=0;i<cloud.points.size();i++)
	    {
	      gx = (int) ((cloud.points[i].x - resp.map.info.origin.position.x)/resp.map.info.resolution);
	      gy = (int) ((cloud.points[i].y - resp.map.info.origin.position.y)/resp.map.info.resolution);
	      if(resp.map.data[gx + gy * resp.map.info.width]==0)
	      {
		temp_cloud.points[j]=cloud.points[i];
		j++;
	      }
	    }
	    clean_cloud.points.resize(j);
	    for(int i=0;i<j;i++)
	      clean_cloud.points[i]=temp_cloud.points[i];
	    
	    scan_pub_.publish(clean_cloud);
	
	}
	
	void dataReceived2(const sensor_msgs::LaserScanConstPtr& laser_scan)
	{
	    sensor_msgs::PointCloud cloud;
	    laser_geometry::LaserProjection projector;
	 
	    projector.transformLaserScanToPointCloud("map",*laser_scan, cloud,*tf_);
	    
	    int tollerance = 5; //pixel, radius
	    bool out=false;
	    sensor_msgs::PointCloud clean_cloud;
	    sensor_msgs::PointCloud temp_cloud;
	    clean_cloud.header.stamp=ros::Time::now();
	    clean_cloud.header.frame_id="map";
	    
	    temp_cloud.points.resize(cloud.points.size());
	    int j=0;
	    
	    for(unsigned i=0;i<cloud.points.size();i++)
	    {
	      for(int x_index=-tollerance;x_index<tollerance+1;x_index++)
	      {
		for(int y_index=-tollerance;y_index<tollerance+1;y_index++)
		{
		  //ROS_INFO("num:%d\theight:%d",y_index+gy,resp.map.info.height);
		  //ROS_INFO("%d %d",gy,y_index);
		  //FIXME: control on the border
		  //if((y_index+gy < resp.map.info.height) && (x_index+gx < resp.map.info.width))
		  //{
		   // ROS_INFO("h");
		   
		     if(this->gridCoords(cloud.points[i].x,cloud.points[i].y) )
		     {
		       if(out=checkRange(x_index,y_index))
			 x_index=y_index=tollerance+1;	 
		     }
		       
		  //}
		}
	      }
	      if(out==false)
	      {
		temp_cloud.points[j]=cloud.points[i];
		j++;
	      }
	      else
		out=false;
		
	    }

	    clean_cloud.points.resize(j);
	    for(int i=0;i<j;i++)
	      clean_cloud.points[i]=temp_cloud.points[i];
	    
	    scan_pub_.publish(clean_cloud);
	
	}
	
	bool gridCoords(float x,float y) {
	  if(x < 0 || y < 0){
	    gx = 0;
	    gy = 0;
	    return false;
	  }
	  gx = (int) ((x - resp.map.info.origin.position.x)/resp.map.info.resolution);
	  gy = (int) ((y - resp.map.info.origin.position.y)/resp.map.info.resolution);

	  if(gx >= resp.map.info.width || gy >= resp.map.info.height){
	    gx = 0;
	    gy = 0;
	    return false;
	  }
	  return true;
	}
	
	bool checkRange(int x, int y)
	{
	  if(resp.map.data[(x+gx) + (y+gy) * resp.map.info.width]==100)
		return true;
	  return false;
	}
	
   protected:
       message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
       tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
       
       //Parameters
       std::string laser_frame_id_, target_frame_id_;
       tf::TransformListener* tf_;
      //test
     ros::Subscriber sub;
     
     nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  int gx,gy,index;   
  
   private:
        ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Publisher scan_pub_;
 };

CheckBackground::CheckBackground():
	  private_nh_("~")
    {
	   
	  //Get Parameters
	  if(!private_nh_.getParam("target_frame_id",target_frame_id_))  
	      private_nh_.param("target_frame_id", target_frame_id_, std::string("map"));
	   
	  if(!private_nh_.getParam("laser_frame_id",laser_frame_id_))  
	      private_nh_.param("laser_frame_id", laser_frame_id_, std::string("base_scan"));
	   
	  ROS_INFO("Requesting the map...");
	  while(!ros::service::call("static_map", req, resp))
	  {
	    ROS_WARN("Request for map failed; trying again...");
	    ros::Duration d(0.5);
	    d.sleep();
	  }
	    ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           resp.map.info.width,
           resp.map.info.height,
           resp.map.info.resolution);
   
	    ROS_INFO("Origin [%f,%f,%f]",resp.map.info.origin.position.x,resp.map.info.origin.position.y,resp.map.info.origin.position.z);
	    ROS_INFO("cel_num:%d",resp.map.data.size());
	    

	   
	   
 	   tf_ = new tf::TransformListener();
 	   laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, laser_frame_id_, 1); //100
 	   laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_,target_frame_id_, 1); //100
 	   laser_scan_filter_->registerCallback(boost::bind(&CheckBackground::dataReceived2,this, _1));
	   scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/MTTDI1/object_point_cloud",1);



      }

int main(int argc, char** argv)
{
  ros::init(argc,argv,"test");
  //ros::NodeHandle nh;
  
  sleep(1); //FIXME
CheckBackground newnode; 
  


ros::spin();
  return 0;
}
