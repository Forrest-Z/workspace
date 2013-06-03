# rewrites report such that
# numFilters ID1 x1 y1 l11 l21 angle1 ID2 x2 y2 l12 l22 angle2 ... 
import math
import pylab
import numpy as np
import matplotlib.pyplot as plt


from pylab import * 
from numpy import * 
from matplotlib import *
from matplotlib.pyplot import *
#from pylab import figure, show, rand
from matplotlib.patches import Ellipse

def plotEllipse(pos,P,edge,face):
    U, s , Vh = svd(P)
    orient = math.atan2(U[1,0],U[0,0])*180/pi
    ellipsePlot = Ellipse(xy=pos, width=2.0*math.sqrt(s[0]), height=2.0*math.sqrt(s[1]), angle=orient,facecolor=face, edgecolor=edge)
    ax = gca()
    ax.add_patch(ellipsePlot);
    return ellipsePlot;

saveFigures = False;
plotEnvironment = False;
        
# load data from file
report = loadtxt("report.txt")


# timestep from which to start the plotting
startstep = 0;

# colors and markers used for plotting
colorsPlot=array(['b','g','r','c','k','m','y','r','b','g','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k']);
markersPlot=array(['s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h']);

dimension = 4;
dimensionMeasSim = 4;
measDimension = 2;
clusterDimension =2;

nbeams = 181;
treshold_env = 0.1;

maxNumComp = 20;
maxNumMeas = 181;
maxNumFilters = 20;
maxNumClusters = 20;
maxRange = 8;
laserPosition  = array([0.0,0.0]);


# get data from report file to seperate variables
counter = 0;
num = 1;
timestep = report[:,counter:counter+num];
counter = counter + num;

num = 1;
numMeas = report[:,counter:counter+num];
counter = counter + num;

num = maxNumMeas;
laserDistance = report[:,counter:counter+num];
counter = counter + num;

num = maxNumMeas;
laserAngle = report[:,counter:counter+num];
counter = counter + num;

num = 1;
numObjectsEstimated = report[:,counter:counter+num];
counter = counter + num;

num = 1;
numFilters = report[:,counter:counter+num];
counter = counter + num;

num = maxNumFilters;
filterID = report[:,counter:counter+num];
counter = counter + num;

num = dimension* maxNumFilters;
estimateStateFiltersX = report[:,counter:counter+num:dimension];
estimateStateFiltersY = report[:,counter+1:counter+num:dimension];
estimateStateFiltersVX = report[:,counter+2:counter+num:dimension];
estimateStateFiltersVY = report[:,counter+3:counter+num:dimension];
counter = counter + num;

num = dimension * dimension* maxNumFilters;
step = dimension * dimension;
estimateCovarianceFiltersXX = report[:,counter:counter+num:step];
estimateCovarianceFiltersXY = report[:,counter+1:counter+num:step];
estimateCovarianceFiltersXVX = report[:,counter+2:counter+num:step];
estimateCovarianceFiltersXVY = report[:,counter+3:counter+num:step];
estimateCovarianceFiltersYX = report[:,counter+dimension:counter+num:step];
estimateCovarianceFiltersYY = report[:,counter+dimension+1:counter+num:step];
estimateCovarianceFiltersYVX = report[:,counter+dimension+2:counter+num:step];
estimateCovarianceFiltersYVY = report[:,counter+dimension+3:counter+num:step];
estimateCovarianceFiltersVXX = report[:,counter+2*dimension:counter+num:step];
estimateCovarianceFiltersVXY = report[:,counter+2*dimension+1:counter+num:step];
estimateCovarianceFiltersVXVX = report[:,counter+2*dimension+2:counter+num:step];
estimateCovarianceFiltersVXVY = report[:,counter+2*dimension+3:counter+num:step];
estimateCovarianceFiltersVYX = report[:,counter+3*dimension:counter+num:step];
estimateCovarianceFiltersVYY = report[:,counter+3*dimension+1:counter+num:step];
estimateCovarianceFiltersVYVX = report[:,counter+3*dimension+2:counter+num:step];
estimateCovarianceFiltersVYVY = report[:,counter+3*dimension+3:counter+num:step];
counter = counter + num;

num = 1;
numClusters = report[:,counter:counter+num];
counter = counter + num;

num = maxNumClusters * clusterDimension;
clusterX = report[:,counter:counter+num:clusterDimension];
clusterY = report[:,counter+1:counter+num:clusterDimension];
counter = counter + num;

num = clusterDimension* clusterDimension* maxNumClusters;
step = clusterDimension * clusterDimension;
clusterCovarianceXX = report[:,counter:counter+num:step];
clusterCovarianceXY = report[:,counter+1:counter+num:step];
clusterCovarianceYX = report[:,counter+clusterDimension:counter+num:step];
clusterCovarianceYY = report[:,counter+clusterDimension+1:counter+num:step];
counter = counter + num;

num = nbeams;
laserDistanceReader = report[:,counter:counter+num];
laserAngleReader = report[:,counter:counter+num];
counter = counter + num;


## get data from report file to seperate variables
#counter = 0;
#num = 1;
#timestep = report[:,counter:counter+num];
#counter = counter + num;
#
#num = 1;
#numFilters = report[:,counter:counter+num];
#counter = counter + num;
#
#num = maxNumFilters;
#filterID = report[:,counter:counter+num];
#counter = counter + num;
#
#num = dimension* maxNumFilters;
#estimateStateFiltersX = report[:,counter:counter+num:dimension];
#estimateStateFiltersY = report[:,counter+1:counter+num:dimension];
#estimateStateFiltersVX = report[:,counter+2:counter+num:dimension];
#estimateStateFiltersVY = report[:,counter+3:counter+num:dimension];
#counter = counter + num;
#
#num = dimension * dimension* maxNumFilters;
#step = dimension * dimension;
#estimateCovarianceFiltersXX = report[:,counter:counter+num:step];
#estimateCovarianceFiltersXY = report[:,counter+1:counter+num:step];
#estimateCovarianceFiltersXVX = report[:,counter+2:counter+num:step];
#estimateCovarianceFiltersXVY = report[:,counter+3:counter+num:step];
#estimateCovarianceFiltersYX = report[:,counter+dimension:counter+num:step];
#estimateCovarianceFiltersYY = report[:,counter+dimension+1:counter+num:step];
#estimateCovarianceFiltersYVX = report[:,counter+dimension+2:counter+num:step];
#estimateCovarianceFiltersYVY = report[:,counter+dimension+3:counter+num:step];
#estimateCovarianceFiltersVXX = report[:,counter+2*dimension:counter+num:step];
#estimateCovarianceFiltersVXY = report[:,counter+2*dimension+1:counter+num:step];
#estimateCovarianceFiltersVXVX = report[:,counter+2*dimension+2:counter+num:step];
#estimateCovarianceFiltersVXVY = report[:,counter+2*dimension+3:counter+num:step];
#estimateCovarianceFiltersVYX = report[:,counter+3*dimension:counter+num:step];
#estimateCovarianceFiltersVYY = report[:,counter+3*dimension+1:counter+num:step];
#estimateCovarianceFiltersVYVX = report[:,counter+3*dimension+2:counter+num:step];
#estimateCovarianceFiltersVYVY = report[:,counter+3*dimension+3:counter+num:step];
#counter = counter + num;
#


result = report[:,0:6*20+1];
print result.shape


for i in range(startstep,len(timestep) ):
    result[i,0] =numFilters[i];
    for comp in range(0 , maxNumFilters):
        P = [[estimateCovarianceFiltersXX[i,comp],estimateCovarianceFiltersXY[i,comp]],[estimateCovarianceFiltersYX[i,comp],estimateCovarianceFiltersYY[i,comp]]]
        print "P"
        print P
        U, s , Vh = svd(P)
        orient = math.atan2(U[1,0],U[0,0])*180/pi
        print "orient"
        print orient
        l1 = math.sqrt(s[0])
        l2 = math.sqrt(s[1])
        x = estimateStateFiltersX[i,comp]
        print "x"
        print x
        y = estimateStateFiltersY[i,comp]
        print "y"
        print y
        result[i,comp*6+1] =filterID[i,comp];
        result[i,comp*6+2] =x;
        result[i,comp*6+3] =y;
        result[i,comp*6+4] =l1;
        result[i,comp*6+5] =l2;
        result[i,comp*6+6] =orient;
# save data to txt file
savetxt("output.txt",result,fmt=('%f'))
