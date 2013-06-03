#include "segment.h"
#include "merge.h"
#include <cstdlib>
int main()
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
temp3[i]=0;
}

//read image
image=imread("sample1.bmp",0);

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
objectNo=1;

//run through pixels considering the closest pixel first (check this)
for (int i=0;i<Npixels;i++)
{
//if pixel is not classified as object yet, seed the pixel
xmax=0;xmin=imageWidth-1;ymax=0;ymin=imageHeight-1;zmax=0;zmin=255,j2=0;

//if (i>25270) {cout<<"man"<<indices[i]/imageWidth<<" "<<indices[i]%imageWidth;cin>>temp2[2];}
   if (pixelClass[indices[i]/imageWidth][indices[i]%imageWidth]==0 && numbers[i]<thresh2)
       {
	//cout<<objectNo<<" "<<indices[i]%imageWidth<<" "<<indices[i]/imageWidth<<" "<<i;cin>>temp2[2];
	checkPixel(indices[i]%imageWidth,indices[i]/imageWidth,pixelClass,xmax,xmin,ymax,ymin,zmax,zmin,objectNo,j2);
	
	//Store details of segmented object location,width,hieght and depth of bb 
        objects[objectNo][0]=xmin;objects[objectNo][1]=xmax;objects[objectNo][2]=ymin;objects[objectNo][3]=ymax;
        objects[objectNo][4]=zmin;objects[objectNo][5]=zmax;
        if (j2>200) 
	    {
	     objectNo++;
	     rectangle(image, cvPoint(xmin,ymin), cvPoint(xmax,ymax), cvScalar(255,0,0), 1);
	    }
        }
}
imwrite("answer.bmp",image);

return 0;
}//end main2
