#include "merge.h"
#include "segment.h"

void main()
{
	Cv::Mat image;
	Cv::Point pt1,pt2;
	//int arrayOne1[2][3] = { {65, 72, 5},{ 105, 55, 2} };
	int arrayTwo[25344],temp2[25344],Npixels=25344;
	int indices[25344];
	int pixelClass[144][176];
	int* arrayOne,temp3;
	//arrayOne = (int*)arrayOne1;
	temp3 = (int*)pixelClass;
//initialize indices
for (int i=0;i<25344;i++)
{
indices[i]=[i];
temp3[i]=0;
}

//read image
image = imread("/home/sid/ros_workspace/project1/src/sample1.bmp",0);
	//merge sort pixel depth values
	arrayOne = (int*)image.data;
	originalImage= (int*)image.data;
mergeSort(arrayOne,indices,arrayTwo, temp2, 25344);

	for (int i = 0; i < 6; i++)
	{
	cout<<indices[i]<<" ";
	cout<<arrayOne[i]<<" ";
	}//end for

int xmax,xmin,ymax,ymin,zmax,zmin,objectNo;
objectNo=0;

//run through pixels considering the closest pixel first (check this)
for (int i=0;i<Npixels;i++)
{

//if pixel is not classified as object yet, seed the pixel
   if (pixelClass[indices[i]/imageWidth][indices[i]%imageWidth]==0)
       checkPixel(indices[i]/imageWidth,indices[i]%imageWidth,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo)

//Store details of segmented object location,width,hieght and depth of bb 
objects[objectNo][0]=xmin;objects[objectNo][1]=xmax;objects[objectNo][2]=ymin;objects[objectNo][3]=ymax;
objects[objectNo][4]=zmin;objects[objectNo][5]=zmax;
objectNo++;

}



}//end main2
