/*
 * getlms.cpp
 *  Created for: Sid at RAMP for lms
 *  Created on : 2012-05-24
 *  Last mod'd : 2012-05-28
 *      Author : Steven Lee
 */

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

// size modifiers
const int numofframes = 1000;
const int grid_size = 2000;

// global declarations
laser_geometry::LaserProjection projector_;
ros::Publisher pub;
sensor_msgs::PointCloud lms_frames[numofframes];
double static objthresh = .5;
int lms_cart[grid_size][grid_size];
int frame_num=0;
int xco, yco, numofpoints;
Mat lms_frame(grid_size, grid_size, CV_8UC1, Scalar(0));
Mat rangeImage = Mat::zeros(grid_size,grid_size,CV_8UC3);

void lmgetCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
// declare pointcloud object
    sensor_msgs::PointCloud cloud;
// convert to cartesian
    projector_.projectLaser(*scan_in, cloud);

// publish cartesian
    pub.publish(cloud);

// calc num of points
    numofpoints = ((scan_in->angle_max) - (scan_in->angle_min)) / (scan_in->angle_increment);

// convert to 1000X1000 matrix
	for (int n=0; n<numofpoints; n++){
		xco = int(grid_size/2) + int(cloud.points[n].x * 100 + int(grid_size/2));
		yco = int(grid_size/2) + int(cloud.points[n].y * 100 + int(grid_size/2));
		if((xco>=0)&&(xco<grid_size)&&(yco>=0)&&(yco<grid_size)){
			lms_cart[xco][yco] = 1;
			rangeImage.at<Vec3f>(yco, xco)={255,255,255};
		}
    }
    
/*/ display for testing    
    for (int m=400; m<600; m++){        // 0, grid_size
        for (int l=400; l<600; l++){    // 0, grid_size
            if (lms_cart[m][l] == 1){
                cout<<"0";
                lms_cart[m][l] = 0;
            }
            else{
                cout<<".";
                lms_cart[m][l] = 0;
            }
        }
        cout<<endl;
    }
    cout<<endl;
*/

// copying frames    
    lms_frames[frame_num] = cloud;
        
    if (frame_num >= numofframes-1){
        frame_num = 0;
    }
    else{
        frame_num++;
    }
    
    
    
	
}

int main( int argc, char** argv ){
    cv::namedWindow(WINDOW,0);
cvResizeWindow("WINDOW",CV_WINDOW_AUTOSIZE);
	  ros::init(argc, argv, "lmgetter");
	  ros::NodeHandle ns;
	  ros::NodeHandle np;
 	  pub  = np.advertise<sensor_msgs::PointCloud>("base_scan_c", 1000);
	  ros::Subscriber sub = ns.subscribe("base_scan", 1, lmgetCallback);
	  cv::imshow(WINDOW, rangeImage);
    	  cv::waitKey(3);
	  ros::spin();
	  return 0;
}


/*	LaserScan.msg variables
    float angle_min; 		// start angle of the scan [rad]
	float angle_max; 		// end angle of the scan [rad]
	float angle_increment; 	// angular distance between measurements [rad]
	float time_increment; 	// time between measurements [seconds]
	float scan_time; 		// time between scans [seconds]
	float range_min; 		// minimum range value [m]
	float range_max; 		// maximum range value [m]
	float ranges[]; 		// range data [m]
	float intensities[];    */


