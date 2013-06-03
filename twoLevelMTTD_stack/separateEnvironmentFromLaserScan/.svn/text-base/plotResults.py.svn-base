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
    show()
    return ellipsePlot

# load data from file
report = loadtxt("report.txt")

dimension = 2;
nbeams = 181;
treshold_env = 0.1;
maxNumComp = 5;

colors=['b' 'g' 'r' 'c' 'm' 'y' 'k'];

counter = 0;

num = 1;
timestep = report[:,counter:counter+num];
counter = counter + num;

num = 1;
numMeasurements = report[:,counter:counter+num];
counter = counter + num;

num = nbeams;
distancesSeparatedObjects = report[:,counter:counter+num];
counter = counter + num;

num = nbeams;
anglesSeparatedObjects = report[:,counter:counter+num];
counter = counter + num;

num = nbeams;
probMeasurementsEnv = report[:,counter:counter+num];
counter = counter + num;

num = nbeams;
laserAngle = report[:,counter:counter+num];
counter = counter + num;

num = nbeams;
laserDistance = report[:,counter:counter+num];
counter = counter + num;



for i in range(1,len(timestep) + 1):
    figure(1);
    #subplot(2,1,1);
    legendEntries = []; 
    legendText = [];
    ax = gca()
    hold(False);
    indices = range(0,nbeams);
    measPlot=plot(laserDistance[i-1,indices] * cos(laserAngle[i-1,indices]),laserDistance[i-1,indices] * sin(laserAngle[i-1,indices]), marker = 'o',  markeredgecolor = 'k', markerfacecolor='k' ,linestyle = 'None');
    legendEntries.append(measPlot)
    legendText.append("measurements")
    #legende = cat(1,legende,{'measurements' });
    hold(True);
    indices = range(0,numMeasurements[i-1]);
    separatedMeasurements=plot(distancesSeparatedObjects[i-1,indices] * cos(anglesSeparatedObjects[i-1,indices]),distancesSeparatedObjects[i-1,indices] * sin(anglesSeparatedObjects[i-1,indices]), marker = '*',  markeredgecolor = 'r', markerfacecolor='r' ,linestyle = 'None');
    legendEntries.append(separatedMeasurements)
    legendText.append("separatedMeasurements")
    title('Results seperator')
    axis([-8,8,-1,8])
    # make legend 
    props = font_manager.FontProperties(size=10)
    lgd = legend(legendEntries,legendText,numpoints=1,prop=props)
    # set axis limits
    ax.set_xlim( (-8.0, 8.0) )
    ax.set_ylim( (-1.0, 8.0) )
    # set labels
    xlabel('x[m]')
    ylabel('y[m]')
    # turn grid on
    grid(True)
    #show()

    #figure(1);
    #subplot(2,1,2);
    #legende = {};
    #hold off;
    #indices_obj = find(probMeasurementsEnv(i,:)<treshold_env);
    #indices_env = find(probMeasurementsEnv(i,:)>=treshold_env);
    #plot(laserDistance(i,indices_env) .* cos(laserAngle(i,indices_env)), laserDistance(i,indices_env) .* sin(laserAngle(i,indices_env)), 'k*');
    #legende = cat(1,legende,{'environment' });
    #hold on;
    #plot(laserDistance(i,indices_obj) .* cos(laserAngle(i,indices_obj)), laserDistance(i,indices_obj) .* sin(laserAngle(i,indices_obj)), 'r*');
    #legende = cat(1,legende,{'objects' });
    #hold on;
    #axis([-8,8,-1,8])
    #legend(legende);

    #pause(0.1);
    while not waitforbuttonpress():
        pass

