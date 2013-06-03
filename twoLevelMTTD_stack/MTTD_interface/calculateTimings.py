import math
import pylab
import numpy as np
import matplotlib.pyplot as plt
import time


from pylab import * 
from numpy import * 
from matplotlib import *
from matplotlib.pyplot import *
#from pylab import figure, show, rand
from matplotlib.patches import Ellipse


saveFigures = False;
automaticAxisLimits = False;
        
# load data from file
timings = loadtxt("timings.txt")
report = loadtxt("report.txt")
numTimesteps = report[:,1].size;

dimension = 4;
measDimension = 2;
clusterDimension =2;

maxNumMeas = 361;
maxNumFilters = 20;
maxNumClusters = 40;

# colors and markers used for plotting
colorsPlot=array(['b','g','r','c','k','m','y','r','b','g','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k']);
markersPlot=array(['s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h']);


# get data from report file to seperate variables

counter = 0;
num = 1;
timestep = report[:,counter:counter+num];
counter = counter + num;

num = 1;
numMeas = report[:,counter:counter+num];
counter = counter + num;

num = maxNumMeas * measDimension;
measurementsObjectsX = report[:,counter:counter+num:measDimension];
measurementsObjectsY = report[:,counter+1:counter+num:measDimension];
counter = counter + num;

num = 1;
numObjectsEstimated = report[:,counter:counter+num];
counter = counter + num;

num = 1;
numFilters = report[:,counter:counter+num];
counter = counter + num;

# get data from timings file
counter = 0;
num = 1;
#timestep
counter = counter + num;

step=6;
end= numTimesteps*6+1;

num = 1;
timePrepare = timings[counter:end:step];
counter = counter + num;

num = 1;
timeClustering = timings[counter:end:step];
counter = counter + num;

num = 1;
timeNumObjects = timings[counter:end:step];
counter = counter + num;

num = 1;
timeUpdateFilters = timings[counter:end:step];
counter = counter + num;

num = 1;
timeReporting = timings[counter:end:step];
counter = counter + num;

num = 1;
timeUpdateHook = timings[counter:end:step];
counter = counter + num;

#calculate where new target enters

diffNumObjects = numObjectsEstimated[1:numObjectsEstimated.size-1] - numObjectsEstimated[2:numObjectsEstimated.size];
newObjectTimings = nonzero(diffNumObjects == -1.0)[0]

figure(1)
legendEntries = []; 
legendText = [];
ax = gca()
hold(False)
ax.set_autoscale_on(False)
#timePreparePlot = plot(timestep,timePrepare,'r*')
#legendEntries.append(timePreparePlot)
#legendText.append("time up to prepare")

#hold(True)

timeClusteringPlot = plot(timestep,timeClustering-timePrepare,'bo')#,markersize='2')
legendEntries.append(timeClusteringPlot)
legendText.append("time for clustering")
hold(True)

timeNumObjectsPlot = plot(timestep,timeNumObjects-timeClustering,'g.')
legendEntries.append(timeNumObjectsPlot)
legendText.append("time for estimating the number of objects")

timeUpdateFiltersPlot = plot(timestep,timeUpdateFilters-timeClustering,'rd')
legendEntries.append(timeUpdateFiltersPlot)
legendText.append("time for data association and updating the filters")

#timeReportingPlot = plot(timestep,timeReporting,'k-')
#legendEntries.append(timeReportingPlot)
#legendText.append("time up to reporting")

timeUpdateHookPlot = plot(timestep,timeUpdateHook,'k*')
legendEntries.append(timeUpdateHookPlot)
legendText.append("total time")

for i in range(0,len(newObjectTimings)):
	print newObjectTimings[i]
	axvline(x=timestep[newObjectTimings[i]],linewidth=1,color='r')
	if i == 0:
		text(timestep[0]+(timestep[newObjectTimings[i]]-timestep[0])*0.5,timeUpdateHook.max()+0.001,'1 target',fontsize=12,rotation='vertical',ha='center',va='top')
	else:
		numObjectsLoc = numObjectsEstimated[newObjectTimings[i],0]
		textLoc = str(int(numObjectsLoc))+" targets"
		text(timestep[newObjectTimings[i-1]]+0.5*(timestep[newObjectTimings[i]]-timestep[newObjectTimings[i-1]]),timeUpdateHook.max()+0.001,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')

numObjectsLoc = numObjectsEstimated[newObjectTimings[i],0]+1
textLoc = str(int(numObjectsLoc))+" targets"
text(timestep[newObjectTimings[i]]+5,timeUpdateHook.max()+0.001,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')


numObjectsLoc =20 
textLoc = str(int(numObjectsLoc))+" targets"
index20=40
axvline(x=40,linewidth=1,color='r')
text(42,timeUpdateHook.max()+0.001,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')

# make legend 
props = font_manager.FontProperties(size=10)
lgd = legend(legendEntries,legendText,numpoints=1,prop=props,loc='lower right')
# set axis square
#axis("scaled")
# set axis limits
#ax.set_ylim( (0, numClusters.max()+1 ) )
# set labels
xlabel('timestep')
ylabel('time[s]')
# turn grid on
grid(True)
#leg = ax.legend(loc='center left', shadow=True, fancybox=True, prop=props)
#leg.get_frame().set_alpha(0.5)
title("Timings")

if saveFigures:
    savefig("figures/timingsAll.png",format='png',dpi=200)

figure(2)
legendEntries = []; 
legendText = [];
ax = gca()
hold(False)
ax.set_autoscale_on(False)
#timePreparePlot = plot(timestep,timePrepare,'r*')
#legendEntries.append(timePreparePlot)
#legendText.append("time up to prepare")



timeUpdateHookPlot = plot(timestep,timeUpdateFilters-timePrepare,'k.')
legendEntries.append(timeUpdateHookPlot)
legendText.append("total time")
hold(True)

#timeClusteringPlot = plot(timestep,timeClustering-timePrepare,'k*')#,markersize='2')
#legendEntries.append(timeClusteringPlot)
#legendText.append("time for clustering")
#hold(True)

for i in range(0,len(newObjectTimings)):
	print newObjectTimings[i]
	axvline(x=timestep[newObjectTimings[i]],linewidth=1,color='r')
	numObjectsLoc = numObjectsEstimated[newObjectTimings[i],0]
	if numObjectsLoc==1:
		textLoc = str(int(numObjectsLoc))+" target"
	else:
		textLoc = str(int(numObjectsLoc))+" targets"
	if i == 0:
		text(timestep[0]+(timestep[newObjectTimings[i]]-timestep[0])*0.5,timeUpdateHook.max()+0.001,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')
	else:
		text(timestep[newObjectTimings[i-1]]+0.5*(timestep[newObjectTimings[i]]-timestep[newObjectTimings[i-1]]),timeUpdateHook.max()+0.001,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')

numObjectsLoc = numObjectsEstimated[newObjectTimings[i],0]+1
textLoc = str(int(numObjectsLoc))+" targets"
text(timestep[newObjectTimings[i]]+5,timeUpdateHook.max()+0.001,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')


numObjectsLoc =20 
textLoc = str(int(numObjectsLoc))+" targets"
index20=63
axvline(x=63,linewidth=1,color='r')
text(65,timeUpdateHook.max()+0.001,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')

# set labels
xlabel('timestep')
ylabel('time[s]')
# turn grid on
grid(True)
#leg = ax.legend(loc='center left', shadow=True, fancybox=True, prop=props)
#leg.get_frame().set_alpha(0.5)
title("Timings")
# make legend 
#props = font_manager.FontProperties(size=10)
#lgd = legend(legendEntries,legendText,numpoints=1,prop=props,loc='lower right')

if saveFigures:
    savefig("figures/timings.png",format='png',dpi=200)


# plot inverse

figure(3)
legendEntries = []; 
legendText = [];
ax = gca()
hold(False)
ax.set_autoscale_on(False)
#timePreparePlot = plot(timestep,timePrepare,'r*')
#legendEntries.append(timePreparePlot)
#legendText.append("time up to prepare")

#hold(True)


timeUpdateHookPlot = plot(timestep,1.0/(timeUpdateFilters-timePrepare),'k.')
hold(True)

for i in range(0,len(newObjectTimings)):
	print newObjectTimings[i]
	axvline(x=timestep[newObjectTimings[i]],linewidth=1,color='r')
	numObjectsLoc = numObjectsEstimated[newObjectTimings[i],0]
	if numObjectsLoc==1:
		textLoc = str(int(numObjectsLoc))+" target"
	else:
		textLoc = str(int(numObjectsLoc))+" targets"
	if i == 0:
		text(timestep[0]+(timestep[newObjectTimings[i]]-timestep[0])*0.5,40,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')
	else:
		text(timestep[newObjectTimings[i-1]]+0.5*(timestep[newObjectTimings[i]]-timestep[newObjectTimings[i-1]]),40,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')

numObjectsLoc = numObjectsEstimated[newObjectTimings[i],0]+1
textLoc = str(int(numObjectsLoc))+" targets"
text(timestep[newObjectTimings[i]]+5,40,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')


numObjectsLoc =20 
textLoc = str(int(numObjectsLoc))+" targets"
index20=63
axvline(x=63,linewidth=1,color='r')
text(65,40,textLoc,fontsize=12,rotation='vertical',ha='center',va='top')

# set axis limits                                                                                                                                                                                              
ax.set_xlim( (-1, timestep.max()) )   
ax.set_ylim( (0, 120) )
#ax = axes([0,timestep.max(),0,150])
# set labels
xlabel('timestep')
ylabel('fps')
# turn grid on
grid(True)
#leg = ax.legend(loc='center left', shadow=True, fancybox=True, prop=props)
#leg.get_frame().set_alpha(0.5)
#title("Frames per second")

if saveFigures:
    savefig("figures/framesPerSecond.png",format='png',dpi=200)
else: 
    show()

averagePrepare = timePrepare.mean()
averageClustering = timeClustering.mean()
averageNumObjects = timeNumObjects.mean()
averageUpdateFilters = timeUpdateFilters.mean()
averageReporting = timeReporting.mean()
averageUpdateHook = timeUpdateHook.mean()

print 'average time needed up to prepare measurements: ', averagePrepare 
print 'average time needed up to cluster measurements: ', averageClustering 
print 'average time needed up to estimate number of objects : ', averageNumObjects 
print 'average time needed up to update the filters : ', averageUpdateFilters 
print 'average time needed up to reporting : ', averageReporting 
print 'average time needed for update hook : ', averageUpdateHook 


print 'average time needed to prepare measurements: ', averagePrepare 
print 'average time needed to cluster measurements: ', averageClustering - averagePrepare
print 'average time needed to estimate number of objects : ', averageNumObjects - averageClustering
print 'average time needed to update the filters : ', averageUpdateFilters - averageNumObjects
print 'average time needed to reporting : ', averageReporting - averageUpdateFilters
print 'average time needed for update hook : ', averageUpdateHook 

print 'average time needed for algorithm : ', averageUpdateFilters-averagePrepare
