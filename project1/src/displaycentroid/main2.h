#include "segment.h"
#include "merge.h"
#include <cstdlib>
void main2(Mat image) //change for .h
{
	int arrayTwo[25344],temp2[25344],Npixels=25344;
        uchar numbers[25344];
	int indices[25344];
	int pixelClass[144][176];
	int *temp3;
	uchar *arrayOne;
	temp3 = (int*)pixelClass;

//initialize indices
for (int i=0;i<25344;i++)
{
indices[i]=i;
temp3[i]=-2;
}

//read image
//Mat image=imread("sample1.bmp",0); (change for .h)
cvtColor(image,image2,CV_GRAY2RGB);
//merge sort pixel depth values
arrayOne = image.data;
originalImage= image.data;
for (int i=0;i<25344;i++)
{
numbers[i]=arrayOne[i];
}

mergeSort(numbers,indices,arrayTwo, temp2, 25344);

/*
	for (int i = 0; i < 25344; i++)
	{
	
        {//cout<<"ha"<<i<<" "<<indices[i]<<" ";
	cout<<int(numbers[i])<<" ";}
	}
*/

int xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2;
objectNo=0;

//run through pixels considering the closest pixel first (check this)
for (int i=0;i<Npixels;i++)
{
//if pixel is not classified as object yet, seed the pixel
xmax=0;xmin=imageWidth-1;ymax=0;ymin=imageHeight-1;zmax=0;zmin=255,j2=0;

//if (i>25270) {cout<<"man"<<indices[i]/imageWidth<<" "<<indices[i]%imageWidth;cin>>temp2[2];}
   if (pixelClass[indices[i]/imageWidth][indices[i]%imageWidth]==-2 && numbers[i]<thresh2 && numbers[i]>thresh3)
       {
	//cout<<objectNo<<" "<<indices[i]%imageWidth<<" "<<indices[i]/imageWidth<<" "<<i;cin>>temp2[2];
	checkPixel(indices[i]%imageWidth,indices[i]/imageWidth,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	
	//Store details of segmented object location,width,hieght and depth of bb 
        objects[objectNo].BoundingBoxMin.x=xmin;objects[objectNo].BoundingBoxMax.x=xmax;objects[objectNo].BoundingBoxMin.y=ymin;objects[objectNo].BoundingBoxMax.y=ymax;
        objects[objectNo].BoundingBoxMin.z=zmin;objects[objectNo].BoundingBoxMax.z=zmax;
        if (j2>600) 
	    {
	     objectNo++;
	     rectangle(image2, cvPoint(xmin,ymin), cvPoint(xmax,ymax), CV_RGB( 255, 255, 255 ), 1);
	    }
        }
}
for (int i2=0;i2<objectNo;i2++)
{
cout<<objects[i2].Centroid.x<<" "<<objects[i2].Centroid.y<<" "<<objects[i2].Centroid.z<<endl;
}
cin>>noOfObjects;
/*
// object correspondance
int count1=0,objectFound;
currentTrace=startTrace;
for(int i=0;i<noOfObjects;i++)
{
	objectFound = 0;
	for(int i1=0;i1<objectNo;i1++)
	{
		if ( (objects[i1].Centroid.x-currentTrace->Centroid.x)*(objects[i1].Centroid.x-currentTrace->Centroid.x) + (objects[i1].Centroid.z-currentTrace->Centroid.z)*(objects[i1].Centroid.z-currentTrace->Centroid.z) < threshCentroid)
		{
		count1++;
		currentTrace->frameNo++;
		currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin=objects[i1].BoundingBoxMin;
		currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax=objects[i1].BoundingBoxMax;
		currentTrace->currentObject[currentTrace->frameNo].Centroid.x=objects[i1].Centroid.x;
		objects[i1].correspond=1;
		objectFound = 1;
		}
		if (objectFound==0) currentTrace->notFound++;
		else
		currentTrace->notFound=0; 
		currentTrace=currentTrace->nextTrace;
	}
}

//storing new objects
noOfObjects=noOfObjects+objectNo-count1;
for (int i1=0;i1<objectNo;i1++)
{
	if(objects[i1].correspond!=1)
	{
	currentTrace->currentObject[0].BoundingBoxMin=objects[i1].BoundingBoxMin;
	currentTrace->currentObject[0].BoundingBoxMax=objects[i1].BoundingBoxMax;
	currentTrace->currentObject[0].Centroid=objects[i1].Centroid;
	currentTrace->color.x=255*rand();currentTrace->color.y=255*rand();currentTrace->color.z=255*rand();
	currentTrace->notfound=0;
	currentTrace->frameNo=0;
	currentTrace->nextTrace = new ObjectTrace;
	currentTrace->nextTrace->prevTrace=currentTrace;
	currentTrace=currentTrace->nextTrace;
	}
}
*/

//imwrite("answer.bmp",image2); change image2

//return 0; change for .h
}
