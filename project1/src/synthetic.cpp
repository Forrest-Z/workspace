#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<iostream>
using namespace std;
using namespace cv;
int main()
{
Mat img1 = Mat(144,176, CV_8UC1, Scalar(0,0,255)); 
//IplImage* img1=cvLoadImage("synthe1.bmp",0);
//Mat img1=imread("synthe1.bmp",0);
uchar* data    = img1.data;
int height     = img1.cols;
int width      = img1.rows;
int step       = img1.rows;
for(int i=0;i<height;i++) for(int j=0;j<width;j++) 
data[i*step+j]=211;

rectangle(img1, cvPoint(10,10), cvPoint(40,40), cvScalar(200,0,0), 1);
circle(img1, cvPoint(100, 100), 30, cvScalar(0,0,0));
ellipse(img1,cvPoint(150,40),Size(20,30),20,0,360,Scalar(0,0,0),-1,8);
int f= floodFill(img1,cvPoint(100,100),0);
cout<<height<<" "<<width<<" "<<step<<" "<<data[100]<<" ";
// if(!cvSaveImage("synthe1.bmp",img1)) cout<<"Could not save";
imwrite("synth2.png",img1);
return 0;
}