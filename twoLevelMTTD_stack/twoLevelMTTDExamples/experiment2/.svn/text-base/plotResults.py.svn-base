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
automaticAxisLimits = False;
        
# load data from file
report = loadtxt("report.txt")
if plotEnvironment:
        environment = loadtxt("environmentEXP4.txt")

# save data to txt file
#savetxt("output.txt",report)

# timestep from which to start the plotting
startstep = 1;

# colors and markers used for plotting
colorsPlot=array(['b','g','r','c','y','m','k','r','b','k','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k']);
markersPlot=array(['s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h','s','d','o','p','h']);

dimension = 4;
dimensionMeasSim = 4;
measDimension = 2;
clusterDimension =2;

nbeams = 181;
treshold_env = 0.1;

maxNumComp = 20;
maxNumMeas = 500;
maxNumFilters = 20;
maxNumClusters = 20;
maxRange = 8;
laserPosition  = array([0.0,0.0]);

# draw environment
if plotEnvironment:
        timestep = environment[0,0]
        envMeas = environment[0,1:nbeams+1]
        angles = pi/180*arange(0,nbeams);
        Xenv = cos(angles)*envMeas;
        Yenv = sin(angles)*envMeas;
        figure(1)
        plot(Xenv,Yenv,'k*')
        
angles = arange(0,pi+pi/361,pi/361);
rangeX = maxRange*cos(angles);
rangeY = maxRange*sin(angles);
rangeVert = array([rangeX,rangeY]).transpose()
        

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
estimateCovarianceFiltersVXVX =report[:,counter+2*dimension+2:counter+num:step];
estimateCovarianceFiltersVXVY =report[:,counter+2*dimension+3:counter+num:step];
estimateCovarianceFiltersVYX = report[:,counter+3*dimension:counter+num:step];
estimateCovarianceFiltersVYY = report[:,counter+3*dimension+1:counter+num:step];
estimateCovarianceFiltersVYVX =report[:,counter+3*dimension+2:counter+num:step];
estimateCovarianceFiltersVYVY =report[:,counter+3*dimension+3:counter+num:step];
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


if automaticAxisLimits:
    measurementsObjectsx=laserDistance * cos(laserAngle);
    measurementsObjectsy = laserDistance * sin(laserAngle);
    measurementsObjectsx = reshape(measurementsObjectsx,(measurementsObjectsx.size,))
    measurementsObjectsy = reshape(measurementsObjectsy,(measurementsObjectsy.size,))
    maxX = measurementsObjectsx[nonzero(abs(measurementsObjectsx) >1e-5)[0]].max()+ 0.5;
    minX = measurementsObjectsx[nonzero(abs(measurementsObjectsx) >1e-5)[0]].min()-0.5;
    maxY = measurementsObjectsy[nonzero(abs(measurementsObjectsy) >1e-5)[0]].max()+0.5;
    minY = measurementsObjectsy[nonzero(abs(measurementsObjectsy) >1e-5)[0]].min()-0.5;
else:
    minX = 20;
    maxX = 40;
    minY = 0;
    maxY = 20;


for i in range(startstep,len(timestep) + 1):
    print i
    rangePlot = Polygon(zip(rangeX,rangeY),facecolor = '0.90',edgecolor='0.0')
    sensorPlot = Rectangle(laserPosition-0.15,0.3,0.3,facecolor='0.5',edgecolor='0.0')

    figure(3)
    #clear(self)
    #suptitle("scan " + str(i)  )
    subplot(121);
    legendEntries = []; 
    legendText = [];
    ax = gca()
    hold(False)
    ax.set_autoscale_on(False)

    #plot environment
    if(plotEnvironment):
        environment = plot(Xenv,Yenv,'k*')
        legendEntries.append(environment)
        legendText.append("environment")
        hold(True)
    
    teller = 0;

    # plot sensor range
    ##ax.add_patch(rangePlot)
    ##legendEntries.append(rangePlot)
    ##legendText.append("sensor range")
    # plot laserscanner position
    ax.add_patch(sensorPlot)
    legendEntries.append(sensorPlot)
    legendText.append("laserscanner")
    # plot measurements
    indices = range(0,numMeas[i-1]);
    measurements=plot(laserDistance[i-1,indices] * cos(laserAngle[i-1,indices]),laserDistance[i-1,indices] * sin(laserAngle[i-1,indices]), marker = '*',  markeredgecolor = 'k', markerfacecolor='k' ,linestyle = 'None');
    legendEntries.append(measurements)
    legendText.append("measurements")
    hold(True)
    # plot target tracks
    teller = 0;
    for comp in range(1 , maxNumFilters+1):
        indicesComp = nonzero(filterID[i-1] == comp)[0]
        if indicesComp.size != 0:
            if i>200:
                indicesCompLoca = nonzero(filterID[i-200:i,:] == comp)[0]
                indicesCompLocb = nonzero(filterID[i-200:i,:] == comp)[1]
                selectedX = estimateStateFiltersX[i-200:i,:];
                selectedY = estimateStateFiltersY[i-200:i,:];
                test = plot(selectedX[indicesCompLoca,indicesCompLocb],selectedY[indicesCompLoca,indicesCompLocb],marker = '.', color = colorsPlot[comp-1] , linestyle = 'None')
            else:
                indicesCompLoca = nonzero(filterID[0:i,:] == comp)[0]
                indicesCompLocb = nonzero(filterID[0:i,:] == comp)[1]
                selectedX = estimateStateFiltersX[0:i,:];
                selectedY = estimateStateFiltersY[0:i,:];
                test = plot(selectedX[indicesCompLoca,indicesCompLocb],selectedY[indicesCompLoca,indicesCompLocb],marker = '.', color = colorsPlot[comp-1] , linestyle = 'None')
            hold(True)
            teller = teller + 1;
    # plot estimated target positions
    teller = 0;
    for comp in range(1 , maxNumFilters+1):
        indicesComp = nonzero(filterID[i-1] == comp)[0]
        if indicesComp.size != 0:
            # target covariances
            P = [[estimateCovarianceFiltersXX[i-1,indicesComp[0]],estimateCovarianceFiltersXY[i-1,indicesComp[0]]],[estimateCovarianceFiltersYX[i-1,indicesComp[0]],estimateCovarianceFiltersYY[i-1,indicesComp[0]]]]
            #ellipsePlot=plotEllipse([estimateStateFiltersX[i-1,indicesComp],estimateStateFiltersY[i-1,indicesComp]],P,'black',colorsPlot[comp-1])
            ellipsePlot=plotEllipse([estimateStateFiltersX[i-1,indicesComp],estimateStateFiltersY[i-1,indicesComp]],P,colorsPlot[comp-1],'none')
            legendEntries.append(ellipsePlot)
            legendText.append("target " + str(comp) + " covariance")
            hold(True)
            # target positions
            test = plot(estimateStateFiltersX[i-1:i,indicesComp] ,estimateStateFiltersY[i-1:i,indicesComp],marker = 'd', color = colorsPlot[comp-1] , linestyle = 'None',label='component',markersize=8)
            legendEntries.append(test)
            legendText.append("target " + str(comp))
            teller = teller + 1;
    # make legend 
    props = font_manager.FontProperties(size=6)
    lgd = legend(legendEntries,legendText,numpoints=1,prop=props,loc='upper right')
    # set axis square
    axis("scaled")
    # set axis limits
    ax.set_xlim( (minX, maxX) )
    ax.set_ylim( (minY, maxY) )
    # set labels
    xlabel('x[m]')
    ylabel('y[m]')
    # turn grid on
    grid(True)
    #leg = ax.legend(loc='center left', shadow=True, fancybox=True, prop=props)
    #leg.get_frame().set_alpha(0.5)
    #show()
    title("targets")

    figure(3)
    subplot(122);
    legendEntries = []; 
    legendText = [];
    ax = gca()
    hold(False)
    ax.set_autoscale_on(False)

    #plot environment
    if(plotEnvironment):
        environment = plot(Xenv,Yenv,'k*')
        legendEntries.append(environment)
        legendText.append("environment")
        hold(True)
    
    teller = 0;

    # plot sensor range
    #ax.add_patch(rangePlot)
    #legendEntries.append(rangePlot)
    #legendText.append("sensor range")
    ## plot laserscanner position
    #ax.add_patch(sensorPlot)
    #legendEntries.append(sensorPlot)
    #legendText.append("laserscanner")
    # plot measurements
    indices = range(0,numMeas[i-1]);
    measurements=plot(laserDistance[i-1,indices] * cos(laserAngle[i-1,indices]),laserDistance[i-1,indices] * sin(laserAngle[i-1,indices]), marker = '*',  markeredgecolor = 'k', markerfacecolor='k' ,linestyle = 'None', markersize = 10);
    legendEntries.append(measurements)
    legendText.append("measurements")
    # plot estimated clusters 
    teller = 0;
    for cluster in range(0 , numClusters[i-1] ):
        # cluster covariances
        P = [[clusterCovarianceXX[i-1,cluster],clusterCovarianceXY[i-1,cluster]],[clusterCovarianceYX[i-1,cluster],clusterCovarianceYY[i-1,cluster]]]
        ellipsePlot=plotEllipse([clusterX[i-1,cluster],clusterY[i-1,cluster]],P,'black',colorsPlot[cluster])
        legendEntries.append(ellipsePlot)
        legendText.append("cluster " + str(cluster+1) + " shape")
        hold(True)
        # cluster positions
        test = plot(clusterX[i-1:i,cluster] ,clusterY[i-1:i,cluster],marker = 'd', color = colorsPlot[cluster] , linestyle = 'None',label='component', markersize = 8)
        legendEntries.append(test)
        legendText.append("cluster " + str(cluster+1))
        teller = teller + 1;
    # make legend 
    props = font_manager.FontProperties(size=6)
    lgd = legend(legendEntries,legendText,numpoints=1,prop=props,loc='upper right')
    # set axis square
    axis("scaled")
    # set axis limits
    ax.set_xlim( (minX, maxX) )
    ax.set_ylim( (minY, maxY) )
    # set labels
    xlabel('x[m]')
    ylabel('y[m]')
    # turn grid on
    grid(True)
    #leg = ax.legend(loc='center left', shadow=True, fancybox=True, prop=props)
    #leg.get_frame().set_alpha(0.5)
    #show()
    title("clusters")

    if saveFigures:
        if i>=1000:
            savefig("figures/estimationGit" +  str(i) +".png",format='png',dpi=200)
        elif i>=100:
            savefig("figures/estimationGit0" +  str(i) +".png",format='png',dpi=200)
        elif i>=10:
            savefig("figures/estimationGit00" +  str(i) +".png",format='png',dpi=200)
        else: # i<10
            savefig("figures/estimationGit000" +  str(i) +".png",format='png',dpi=200)
    else:
        while not waitforbuttonpress():
            pass
#





