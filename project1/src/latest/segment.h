#include <iostream>
#include <fstream>
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
typedef unsigned char uchar;
const int imageWidth=176;
const int imageHeight=144;
const int thresh1=3;
const int thresh2=240;
const int thresh3=20;
const int threshCentroid=300;
const int mask=5;
const int objectTraceSize=100;
struct point
{
int x,y,z;
};
struct Objects
{
Point3i Centroid;
Point3i BoundingBoxMin;
Point3i BoundingBoxMax;
int correspond;
};
struct ObjectTrace
{
Objects currentObject[objectTraceSize];
Point3i color;
int frameNo,tenscounter;
int notFound;
ObjectTrace *nextTrace;
ObjectTrace *prevTrace;
};
fstream outfile;
VideoWriter writer("/home/siddharth/out.avi",CV_FOURCC('P','I','M','1'),
                           25,cvSize(imageWidth,imageHeight),1);
int noOfObjects=0,cintemp;
Objects objects[30];
uchar *originalImage;
Mat image2;
Mat image3=Mat::zeros(thresh2,176,CV_8UC3);
ObjectTrace *currentTrace;
ObjectTrace *startTrace;


void checkPixel(int x,int y,int pixelClass[144][176],int &xmax,int &xmin,int &ymax,int &ymin,int &zmax,int &zmin,int objectNo,int &j2)
{
int xnew,ynew,D=0,nd=0;
j2++;
pixelClass[y][x]=objectNo;

//calculate centroid
objects[objectNo].Centroid.x=( objects[objectNo].Centroid.x*(j2-1)+x )/j2;
objects[objectNo].Centroid.y=( objects[objectNo].Centroid.y*(j2-1)+y )/j2;
objects[objectNo].Centroid.z=( objects[objectNo].Centroid.z*(j2-1)+originalImage[y*imageWidth+x] )/j2;

//calculate D
for (int iy=y-mask/2;iy<=y+mask/2;iy++)
{
for (int ix=x-mask/2;ix<=x+mask/2;ix++)
{
if (pixelClass[iy][ix]==objectNo) {nd++;D=(D + originalImage[iy*imageWidth+ix]);}
}
}
D=D/nd;

//if(j2%100==0){cout<<"haha"<<x<<" "<<y<<" "<<D<<" "<<j2;cin>>j;}
//imwrite("answer.bmp",image);
if (x>xmax) xmax=x;
if (y>ymax) ymax=y;
if (x<xmin) xmin=x;
if (y<ymin) ymin=y;
if (originalImage[y*imageWidth+x]<zmin) zmin=originalImage[y*imageWidth+x];
if (originalImage[y*imageWidth+x]>zmax) zmax=originalImage[y*imageWidth+x];
//originalImage[y*imageWidth+x]=255;//delete
xnew=x-1;ynew=y;
if (xnew>=0 && pixelClass[ynew][xnew]==-2) 
    {      	    
     if ( fabs( int(originalImage[ynew*imageWidth+xnew])-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }
xnew=x;ynew=y;
xnew=x;ynew=y-1;
if (ynew>=0 && pixelClass[ynew][xnew]==-2) 
    { 
     if ( fabs( originalImage[ynew*imageWidth+xnew]-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }
xnew=x;ynew=y;
xnew=x+1;ynew=y;
if (xnew<176 && pixelClass[ynew][xnew]==-2) 
    { 
     if ( fabs( originalImage[ynew*imageWidth+xnew]-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }
xnew=x;ynew=y;
xnew=x;ynew=y+1;
if (ynew<144 && pixelClass[ynew][xnew]==-2) 
    { 
     if ( fabs( originalImage[ynew*imageWidth+xnew]-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }

}


