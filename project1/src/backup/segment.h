#include <iostream>
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
const int thresh1=2;
const int thresh2=100;
const int mask=10;
int objects[100][10];
uchar *originalImage;
Mat image;

void checkPixel(int x,int y,int pixelClass[144][176],int &xmax,int &xmin,int &ymax,int &ymin,int &zmax,int &zmin,int objectNo,int &j2)
{
int xnew,ynew,D=0,j,nd=0;
j2++;
pixelClass[y][x]=objectNo;
//calculate D
for (int iy=y-mask/3;iy<=y+mask/3;iy++)
{
for (int ix=x-mask/3;ix<=x+mask/3;ix++)
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
if (xnew>=0 && pixelClass[ynew][xnew]==0) 
    {      	    
     if ( fabs( int(originalImage[ynew*imageWidth+xnew])-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }
xnew=x;ynew=y;
xnew=x;ynew=y-1;
if (ynew>=0 && pixelClass[ynew][xnew]==0) 
    { 
     if ( fabs( originalImage[ynew*imageWidth+xnew]-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }
xnew=x;ynew=y;
xnew=x+1;ynew=y;
if (xnew<176 && pixelClass[ynew][xnew]==0) 
    { 
     if ( fabs( originalImage[ynew*imageWidth+xnew]-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }
xnew=x;ynew=y;
xnew=x;ynew=y+1;
if (ynew<144 && pixelClass[ynew][xnew]==0) 
    { 
     if ( fabs( originalImage[ynew*imageWidth+xnew]-D ) <= thresh1 )
	{
         checkPixel(xnew,ynew,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	}
    }

}
//if (ynew<176 && pixelClass[ynew][xnew]==0) 

