import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as mpl
from matplotlib.animation import FuncAnimation
from data_loader import DataLoader, DroneTrajectoryData, LidarSweepData
import math
import scipy.optimize as scimin

class Optimizer():
    def __init__(self):
        self.optimizer = None

    def getBoundaryDistance(self):

        return 0

def covertDistanceToEuclidLocal(sweep_data, drone_pos=None):
    ret = []
    sweep_data = np.array(sweep_data)
    
    for i in range(sweep_data.shape[0]):
        x = sweep_data[i, 1] * np.cos(np.radians(sweep_data[i, 0])) * 0.001
        y = sweep_data[i, 1] * np.sin(np.radians(sweep_data[i, 0])) * 0.001
        ret.append([x, y])
        
    ret = np.array(ret)

    return ret

drone_data = DroneTrajectoryData('FlightPath.csv')
lidar_data = LidarSweepData('LIDARPoints.csv')

upper_bounds = []
for i in range(len(lidar_data.sweep_data_raw)):
    lidar_data_local = covertDistanceToEuclidLocal(lidar_data.sweep_data_raw[i])
    mean_upper_bound = np.mean(lidar_data_local[i, 1], dtype=np.float64)
    upper_bounds.append(mean_upper_bound)


# datax=numpy.array([1,2,3,4,5]) # data coordinates
# datay=numpy.array([2.95,6.03,11.2,17.7,26.8])
datax=lidar_data.scan_ID[:3]
datay=np.array(upper_bounds[:3])
constraintmaxx=np.array([0]) # list of maximum constraints
constraintmaxy=np.array([1.2])

# least square fit without constraints
def fitfunc(x,p): # model $f(x)=a x^2+c
    a,c=p
    return c+a*x**2
def residuals(p): # array of residuals
    return datay-fitfunc(datax,p)
p0=[1,2] # initial parameters guess
pwithout,cov,infodict,mesg,ier=scimin.leastsq(residuals, p0,full_output=True) #traditionnal least squares fit

# least square fir with constraints
def sum_residuals(p): # the function we want to minimize
    return sum(residuals(p)**2)
def constraints(p): # the constraints: all the values of the returned array will be >=0 at the end
    return constraintmaxy-fitfunc(constraintmaxx,p)
pwith=scimin.fmin_slsqp(sum_residuals,pwithout,f_ieqcons=constraints) # minimization with constraint

# plotting
ax=mpl.figure().add_subplot(1,1,1)
ax.plot(datax,datay,ls="",marker="x",color="blue",mew=2.0,label="Datas")
ax.plot(constraintmaxx,constraintmaxy,ls="",marker="x",color="red",mew=2.0,label="Max points")
morex=np.linspace(0,len(datax)+1,100)
ax.plot(morex,fitfunc(morex,pwithout),color="blue",label="Fit without constraints")
ax.plot(morex,fitfunc(morex,pwith),color="red",label="Fit with constraints")
ax.legend(loc=2)
mpl.show()
