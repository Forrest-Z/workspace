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

# load data from file
report = loadtxt("RBBM.out")

counter = 0;
num = 1;
z = report[:,counter:counter+num];
counter = counter + num;

num = 1;
angle = report[:,counter:counter+num];
counter = counter + num;

num = 1;
prob = report[:,counter:counter+num];
counter = counter + num;

figure;
plot(z,prob);
xlabel('z[m]')
ylabel('prob')
title('RBBM')
show()

