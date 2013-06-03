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

saveFigures = False;

# load data from file
report = loadtxt("report.txt")
environment = loadtxt("cpf/environment.txt")

dimension = 4;
nbeams = 181;
maxNumComp = 10;
maxNumMeas = nbeams;
radiusState = 0.17;
maxRange = 8.0;
laserPosition  = array([0.0,0.0]);

# draw environment
timestep = environment[0,0]
envMeas = environment[0,1:nbeams+1]
angles = pi/180*arange(0,nbeams);
Xenv = cos(angles)*envMeas;
Yenv = sin(angles)*envMeas;

angles = arange(0,pi+pi/361,pi/361);
rangeX = maxRange*cos(angles);
rangeY = maxRange*sin(angles);
rangeVert = array([rangeX,rangeY]).transpose()

pyplot.figure(1)
pyplot.plot(Xenv,Yenv,'k*')


colors=['b' 'g' 'r' 'c' 'm' 'y' 'k'];

counter = 0;

num = 1;
timestep = report[:,counter:counter+num];
counter = counter + num;

num = 1;
numComp = report[:,counter:counter+num];
counter = counter + num;

# number of component is assumed to be fixed
nComp = numComp[0];

num = maxNumComp * dimension;
step = dimension;
compX = report[:,counter:counter+num:step];
compY = report[:,counter+1:counter+num:step];
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

for i in range(1,len(timestep) + 1):
    print i
    rangePlot = Polygon(zip(rangeX,rangeY),facecolor = '0.90',edgecolor='0.0')
    sensorPlot = Rectangle(laserPosition-0.15,0.3,0.3,facecolor='0.5',edgecolor='0.0')
    figure(1);
    legendEntries = []; 
    legendText = [];
    ax = gca()
    hold(False)
    environment=plot(Xenv,Yenv,'k*')
    legendEntries.append(environment)
    legendText.append("environment")
    hold(True)
    ax.add_patch(rangePlot)
    legendEntries.append(rangePlot)
    legendText.append("sensor range")
    ax.add_patch(sensorPlot)
    legendEntries.append(sensorPlot)
    legendText.append("laserscanner")
    print "laserscanner plot"

    indices = range(0,numMeas[i-1]);
    measurements=plot(laserDistance[i-1,indices] * cos(laserAngle[i-1,indices]),laserDistance[i-1,indices] * sin(laserAngle[i-1,indices]), marker = '*',  markeredgecolor = 'r', markerfacecolor='r' ,linestyle = 'None');
    legendEntries.append(measurements)
    legendText.append("measurements")
    print "measurements plot"
    hold(True)

    teller = 0;
    for comp in range(1 , numComp[i-1]+1):
        print comp
        # cluster shapes
        indices = teller;
        P = [[(2.0*radiusState)**2,0],[0,(2.0*radiusState)**2]]
        ellipsePlot=plotEllipse([compX[i-1,indices],compY[i-1,indices]],P,'black','0.3')
        print "target plot"
        teller = teller + 1;    
        hold(True)
    legendEntries.append(ellipsePlot)
    legendText.append("targets")
    print "targets plot"
    # make legend 
    props = font_manager.FontProperties(size=10)
    lgd = legend(legendEntries,legendText,numpoints=1,prop=props)
    # set axis limits
    ax.set_xlim( (-9.0, 9.0) )
    ax.set_ylim( (-1.0, 8.5) )
    # set labels
    xlabel('x[m]')
    ylabel('y[m]')
    # turn grid on
    grid(True)
    if saveFigures:
        if i>=1000:
            savefig("figures/measSim" +  str(i) +".png",format='png',dpi=200)
        elif i>=100:
            savefig("figures/measSim0" +  str(i) +".png",format='png',dpi=200)
        elif i>=10:
            savefig("figures/measSim00" +  str(i) +".png",format='png',dpi=200)
        else: # i<10
            savefig("figures/measSim000" +  str(i) +".png",format='png',dpi=200)
    else:
       while not waitforbuttonpress():
           pass
