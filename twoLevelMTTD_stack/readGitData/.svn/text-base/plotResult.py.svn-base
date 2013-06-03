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
    orient = math.atan2(U[1,0],U[0,0])
    ellipsePlot = Ellipse(xy=pos, width=math.sqrt(s[0]), height=math.sqrt(s[1]), angle=orient,facecolor=face, edgecolor=edge)
    ax = gca()
    ax.add_patch(ellipsePlot);
    #show()
    return ellipsePlot
        
# load data from file
report = loadtxt("report.txt")

# save data to txt file
#savetxt("output.txt",report)

# colors and markers used for plotting
colorsPlot=array(['b','g','r','c','m','y','k','b','g','r','c','m','y','k','b','g','r','c','m','y','k']);
markersPlot=array(['s','d','o','p','h','d','o','p','h','s',]);

dimension = 2;

maxNumMeas = 181

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

num = maxNumMeas * dimension;
measurementsObjectsx = report[:,counter:counter+num:dimension];
measurementsObjectsy = report[:,counter+1:counter+num:dimension];


maxX = measurementsObjectsx[nonzero(abs(measurementsObjectsx) >1e-5)[0]].max();
minX = measurementsObjectsx[nonzero(abs(measurementsObjectsx) >1e-5)[0]].min();
maxY = measurementsObjectsy[nonzero(abs(measurementsObjectsy) >1e-5)[0]].max();
minY = measurementsObjectsy[nonzero(abs(measurementsObjectsy) >1e-5)[0]].min();

for i in range(1,len(timestep) + 1):
    print i
    figure(1)
    #subplot(211);
    legendEntries = []; 
    legendText = [];
    ax = gca()
    hold(False)
    ax.set_autoscale_on(False)

    plot([0,0],'s',markeredgecolor='k',markerfacecolor='0.5',markersize=10);
    indices = range(0,numMeas[i-1]);
    measurements=plot(measurementsObjectsx[i-1,indices] ,measurementsObjectsy[i-1,indices] , marker = 'o',  markeredgecolor = 'k', markerfacecolor='k' ,linestyle = 'None');
    hold(True)
    legendEntries.append(measurements)
    legendText.append("measurements cartesian read from file")
    measurementsPOL=plot(laserDistance[i-1,indices] * cos(laserAngle[i-1,indices]), laserDistance[i-1,indices] * sin(laserAngle[i-1,indices]), marker = '*',  markeredgecolor = 'r', markerfacecolor='r' ,linestyle = 'None');
    legendEntries.append(measurementsPOL)
    legendText.append("measurements polar")
    # make legend 
    props = font_manager.FontProperties(size=10)
    lgd = legend(legendEntries,legendText,numpoints=1,prop=props,loc=0)
    # set axis limits
    ax.set_xlim( (minX, maxX) )
    ax.set_ylim( (minY, maxY) )
    #axis("scaled")
    # set labels
    xlabel('x[m]')
    ylabel('y[m]')
    #axis("scaled")
    # turn grid on
    grid(True)
    #show()
    if i>=1000:
        savefig("figures/measurementsGIT" +  str(i) +".png",format='png',dpi=200)
    elif i>=100:
        savefig("figures/measurementsGIT0" +  str(i) +".png",format='png',dpi=200)
    elif i>=10:
        savefig("figures/measurementsGIT00" +  str(i) +".png",format='png',dpi=200)
    else: # i<10
        savefig("figures/measurementsGIT000" +  str(i) +".png",format='png',dpi=200)
    #while not waitforbuttonpress():
    #    pass
#

