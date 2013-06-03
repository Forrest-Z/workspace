


from pylab import * 
from numpy import * 
from matplotlib import *
from matplotlib.pyplot import *
#from pylab import figure, show, rand
from matplotlib.patches import Ellipse

from mpl_toolkits.mplot3d import Axes3D

# load data from file
report = loadtxt("RBBM181.out")

counter = 0;
num = 1;
angle = report[:,counter:counter+num] * pi/180;
counter = counter + num;

num = 1;
z = report[:,counter:counter+num];
counter = counter + num;

num = 1;
prob = report[:,counter:counter+num];
counter = counter + num;

fig = figure()
ax = Axes3D(fig)
xs = z * cos(angle)
ys  = z * sin(angle)
zs =  prob
scatterPlot = ax.scatter3D(xs, ys, zs, s=20,c= zs,cmap=cm.jet)
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_zlabel('prob')
cbar = fig.colorbar(scatterPlot,shrink=0.9,extend='max') 
#fig.colorbar(scatterPlot, shrink=0.5, aspect=5)

fig = figure()
scatter(xs, ys, s=15,c = zs,cmap=cm.jet,edgecolors='none')
xlabel('x[m]')
ylabel('y[m]')
colorbar(extend='max')

show()


