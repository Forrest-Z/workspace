#include "project2_func.h"


void lmgetCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
// calc num of points
    numofpoints = ((scan_in->angle_max) - (scan_in->angle_min)) / (scan_in->angle_increment);
    laserColors = new color[numofpoints];
    for (int i=0;i<numofpoints;i++)
	{
	   laserColors[i].r=0;laserColors[i].b=0;laserColors[i].g=0;
	}

// segment into objects
segment(scan_in);

// display point cluod
   Mat rangeImage = Mat::zeros(grid_size,grid_size,CV_8UC3);
   // declare pointcloud object
      sensor_msgs::PointCloud cloud;
   // convert to cartesian
      projector_.projectLaser(*scan_in, cloud);
   // publish cartesian
      pub.publish(cloud);
   // convert to 1000X1000 matrix
	for (int n=0; n<numofpoints; n++)
	  {
		xco = int(grid_size/2) - int(cloud.points[n].x * 100/2);
		//xco = int(cloud.points[n].x * 100/2);
		//if(int(cloud.points[n].y * 100/2 < 0){
		//int(cloud.points[n].y * 100/2
		//}
		//yco = int(cloud.points[n].y * 100/2);
		yco = int(grid_size/2) - int(cloud.points[n].y * 100/2);
		
		//xco = int(cloud.points[n].x * 100/2);
		//yco = int(cloud.points[n].y * 100/2);
		if((xco>=0)&&(xco<grid_size)&&(yco>=0)&&(yco<grid_size))
		  {
			rangeImage.at<Vec3b>(xco, yco)={(uchar)laserColors[n].r,(uchar)laserColors[n].g,(uchar)laserColors[n].b};			
		  }  
    	  }
   // show rangeImage
	cv::imshow(WINDOW, rangeImage);
    	  cv::waitKey(3);  
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
    cv::namedWindow(WINDOW,CV_WINDOW_AUTOSIZE);
	  ros::init(argc, argv, "lmgetter");
	  ros::NodeHandle ns;
	  ros::NodeHandle np;
 	  pub  = np.advertise<sensor_msgs::PointCloud>("base_scan_c", 1000);
	  ros::Subscriber sub = ns.subscribe("base_scan", 1, lmgetCallback);
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


