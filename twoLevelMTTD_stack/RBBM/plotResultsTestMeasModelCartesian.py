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
from mpl_toolkits.mplot3d import Axes3D

# load data from file
report = loadtxt("RBBMCartesian.out")

counter = 0;
num = 1;
x = report[:,counter:counter+num];
counter = counter + num;

num = 1;
y = report[:,counter:counter+num];
counter = counter + num;

num = 1;
prob = report[:,counter:counter+num];
counter = counter + num;

xs = x 
ys = y
zs = prob

fig = figure()
scatter(xs, ys, s=15,c = zs,cmap=cm.gray,edgecolors='none')
xlabel('x[m]')
ylabel('y[m]')
colorbar(extend='max')
title("Full scan laser scanner model")
axes().set_aspect('equal')


#fig = figure()
#ax = Axes3D(fig)
#scatterPlot = ax.scatter3D(xs, ys, zs, s=20,c= zs,cmap=cm.jet)
#ax.set_xlabel('x[m]')
#ax.set_ylabel('y[m]')
#ax.set_zlabel('prob')
#cbar = fig.colorbar(scatterPlot,shrink=0.9,extend='max') 

show()
