#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
static const char WINDOW[] = "Image window";
const int numofframes = 1000;
const int grid_size = 1000;
double static depthThresh = .2;
const int noiseTolerence = 5;
const int pixelNoThresh =10;
struct point
{
float depth;
int yaw;
};
struct color
{
int r,g,b;
};
struct objects
{
int objectno;
point startPoint,endPoint;
float minDepth,maxDepth;
color colour;
};


// global declarations
laser_geometry::LaserProjection projector_;
ros::Publisher pub;
sensor_msgs::PointCloud lms_frames[numofframes]; 
int frame_num=0;
int xco, yco, numofpoints;
color *laserColors;
objects scannedObjects[30];


//functions
void segment(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
int objectno=0,nonMatches,count=0;
scannedObjects[objectno].startPoint.depth=scan_in->ranges[0];
scannedObjects[objectno].startPoint.yaw=0;
scannedObjects[objectno].minDepth=20;
scannedObjects[objectno].maxDepth=0;
scannedObjects[objectno].colour.r=rand()%255;
scannedObjects[objectno].colour.b=rand()%255;
scannedObjects[objectno].colour.g=rand()%255;
for (int i=0;i<numofpoints-1;i++)
  {
	if(fabs(scan_in->ranges[i]-scan_in->ranges[i+1])<depthThresh)
		{
		   count++;
		   if(scannedObjects[objectno].minDepth>scan_in->ranges[i+1]) scannedObjects[objectno].minDepth=scan_in->ranges[i+1];
		   if(scannedObjects[objectno].maxDepth<scan_in->ranges[i+1]) scannedObjects[objectno].maxDepth=scan_in->ranges[i+1];
		   laserColors[i].r=scannedObjects[objectno].colour.r;
		   laserColors[i].g=scannedObjects[objectno].colour.g;
		   laserColors[i].b=scannedObjects[objectno].colour.b;
		}    
	else
	   {
	     nonMatches=0;
	     for(int j=i+2;j<i+2+noiseTolerence && i+2+noiseTolerence<numofpoints;j++)
		if(fabs(scan_in->ranges[i]-scan_in->ranges[j])>depthThresh) nonMatches++;
	     if (nonMatches > noiseTolerence-2)	
		{
		   scannedObjects[objectno].endPoint.depth=scan_in->ranges[i];
		   scannedObjects[objectno].endPoint.yaw=i;
		   if(count>pixelNoThresh) objectno++;
		   count=0;
		   scannedObjects[objectno].startPoint.depth=scan_in->ranges[i+1];
		   scannedObjects[objectno].startPoint.yaw=i+1;
		   scannedObjects[objectno].colour.r=rand()%255;
		   scannedObjects[objectno].colour.b=rand()%255;
		   scannedObjects[objectno].colour.g=rand()%255;
		}
		
	   }
	

  }
}
