#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "main2.h"
#include <time.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const char WINDOW2[] = "Projected Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("swissranger/distance/image_raw", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW,0);
cvResizeWindow("WINDOW",17600,14400);
  cv::namedWindow(WINDOW2,0);
cvResizeWindow("WINDOW2",176,200);
cvMoveWindow(WINDOW,200,100);
cvMoveWindow(WINDOW2,700,100);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOW2);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//bool f = imwrite("/home/sid/ros_workspace/project1/src/sample1.bmp",cv_ptr->image);
// call main2
main2(cv_ptr->image);
 //   if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
 //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

/* get the image data
  int height    = cv_ptr->image.rows;
  int width     = cv_ptr->image.cols;
  int step      = cv_ptr->image.step;
  uchar* data      = (uchar *)cv_ptr->image.data;

for(int i=0;i<height;i++) for(int j=0;j<width;j++) 
   data[i*step+j]=255-data[i*step+j];
*/
    cv::imshow(WINDOW, image2);
    cv::imshow(WINDOW2, image3);
    cv::waitKey(3);
//    cout<<clock()/(float)CLOCKS_PER_SEC<<" ";
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  startTrace=new ObjectTrace;
  srand(time(NULL));
cout<<writer.open("/home/siddharth/out.avi",CV_FOURCC('P','I','M','1'),
                           25,cvSize(imageWidth,imageHeight),1);
  ros::init(argc, argv, "olis_image_converter");

  ImageConverter ic;
  ros::spin();
  return 0;
}

