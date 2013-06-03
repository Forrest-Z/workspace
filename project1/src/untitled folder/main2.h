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
	if (zmin<thresh3) zmin=thresh3; if (zmax>thresh2) zmax=thresh2;
        if(objects[objectNo].Centroid.z<thresh3) zmin=thresh3; if (objects[objectNo].Centroid.z>thresh2) zmax=thresh2;
        objects[objectNo].BoundingBoxMin.x=xmin;objects[objectNo].BoundingBoxMax.x=xmax;objects[objectNo].BoundingBoxMin.y=ymin;objects[objectNo].BoundingBoxMax.y=ymax;
        objects[objectNo].BoundingBoxMin.z=zmin;objects[objectNo].BoundingBoxMax.z=zmax;
	objects[objectNo].correspond=0;
        if (j2>1000) 
	    {
	     objectNo++;
	  //   rectangle(image2, cvPoint(xmin,ymin), cvPoint(xmax,ymax), CV_RGB( 255, 255, 255 ), 1);
	    }
        }
}

// object correspondance
int count1=0,objectFound,minindex=0;
float mindist=50000,dist;
bool overlap;
point bbmax,bbmin;
ObjectTrace *deleteTrace;
currentTrace=startTrace;
for(int i=0;i<noOfObjects;i++)
{

	//delete old bb image 
	rectangle(image3, cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.z), cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.z), CV_RGB( 0, 0, 0 ), 1);

   //delete objects not found in 100 frames
	if ( currentTrace->notFound == 100)
	{
	if(currentTrace==startTrace)
	{startTrace=currentTrace->nextTrace;delete(currentTrace);currentTrace=startTrace;}
	else
	{	
	currentTrace->prevTrace->nextTrace=currentTrace->nextTrace;
	currentTrace->nextTrace->prevTrace=currentTrace->prevTrace;
	deleteTrace=currentTrace;
	currentTrace=currentTrace->nextTrace;
	delete (deleteTrace);
	}
	noOfObjects--;i--;
	continue;
	}

	objectFound = 0;
	mindist=50000;
	for(int i1=0;i1<objectNo;i1++)
	{
		overlap =0;
		dist=(objects[i1].Centroid.z-currentTrace->currentObject[currentTrace->frameNo].Centroid.z)*(objects[i1].Centroid.z-currentTrace->currentObject[currentTrace->frameNo].Centroid.z);
		//testing overlap
		bbmax.x=currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.x;
		bbmin.x=currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.x;
		bbmax.y=currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.y;
		bbmin.y=currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.y;
		if(objects[i1].BoundingBoxMin.x<bbmax.x && objects[i1].BoundingBoxMax.x>bbmin.x) 
		if(objects[i1].BoundingBoxMin.y<bbmax.y && objects[i1].BoundingBoxMax.y>bbmin.y) 
		overlap =1; 
		if (dist<mindist && overlap==1) {mindist=dist;minindex=i1;}
        }
        if (mindist<threshCentroid && objects[minindex].correspond==0)
		{
		count1++;
		currentTrace->frameNo++;
		if (currentTrace->frameNo==objectTraceSize) currentTrace->frameNo=0;
		currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin=objects[minindex].BoundingBoxMin;
		currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax=objects[minindex].BoundingBoxMax;
		currentTrace->currentObject[currentTrace->frameNo].Centroid=objects[minindex].Centroid;
		objects[minindex].correspond=1;
		objectFound = 1;
		}
		if (objectFound==0) currentTrace->notFound++;
		else
		currentTrace->notFound=0; 
     
	//display old objects
        rectangle(image2, cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.y), cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.y), CV_RGB( currentTrace->color.x, currentTrace->color.y, currentTrace->color.z ), 1);	
	rectangle(image3, cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.z), cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.z), CV_RGB( currentTrace->color.x, currentTrace->color.y, currentTrace->color.z ), 1);	

	currentTrace=currentTrace->nextTrace;
}

//storing new objects
noOfObjects=noOfObjects+objectNo-count1;
//cout<<noOfObjects<<" "<<count1;
for (int i1=0;i1<objectNo;i1++)
{
	if(objects[i1].correspond!=1)
	{
	//delete older bb image
	rectangle(image3, cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.z), cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.z), CV_RGB( 0, 0, 0 ), 1);	

	currentTrace->currentObject[0].BoundingBoxMin=objects[i1].BoundingBoxMin;
	currentTrace->currentObject[0].BoundingBoxMax=objects[i1].BoundingBoxMax;
	currentTrace->currentObject[0].Centroid=objects[i1].Centroid;
	currentTrace->color.x=rand()%255;currentTrace->color.y=rand()%255;currentTrace->color.z=rand()%255;
	currentTrace->notFound=0;
	currentTrace->frameNo=0;
	currentTrace->nextTrace = new ObjectTrace;
	currentTrace->nextTrace->prevTrace=currentTrace;
	//display new objects
        rectangle(image2, cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.y), cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.y), CV_RGB( currentTrace->color.x, currentTrace->color.y, currentTrace->color.z ), 1);	
	rectangle(image3, cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.z), cvPoint(currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.x,currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMax.z), CV_RGB( currentTrace->color.x, currentTrace->color.y, currentTrace->color.z ), 1);	

	currentTrace=currentTrace->nextTrace;
	}
}
outfile.open("debugout.txt", fstream::out | fstream::app);
currentTrace=startTrace;
for (int i2=0;i2<noOfObjects;i2++)
{
outfile<<currentTrace->currentObject[currentTrace->frameNo].BoundingBoxMin.z<<" ";
currentTrace=currentTrace->nextTrace;
}
outfile<<endl;
outfile.close();


//imwrite("answer.bmp",image2); change image2

//return 0; change for .h
}
