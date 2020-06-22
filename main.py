import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_loader import DataLoader, DroneTrajectoryData, LidarSweepData
import math
import csv
import scipy.optimize as opt

class Optimizer():
    def __init__(self):
        self.optimizer = None

    def getBoundaryDistance(self):

        return 0

def helper():
    print("This is assignment from Socville. Which contains functions included:\n1. Drone Lidar data reading & visualizing.\n2. Mapping Lidar data & save result.")
    print("Augments:")
    print("Assign drone data: --drone_data <drone_data_path>")
    print("Assign lidar data: --lidar_data <lidar_data_path>")
    print("Run visualizer: --run_visualize True")
    print("Set Sweep ID to Visualizer: --data_to_show <scan_ID_list> i.e. [0,1,2,3]")
    print("Run Mapping: --run_mapping True")
    print("Save Image--save_result")
    print("\n")

def drawDronePath(data):
    x = data.drone_position[:,0]
    y = data.drone_position[:,1]
    n = data.pose_ID

    plt.style.use('seaborn-pastel')
    
    plt.plot(x,y)
    plt.scatter(x, y, s=20, edgecolors='none', c='red')

    for i,j,k in zip(x,y,n):
        label = k
        plt.annotate(label, # this is the text
                    (i,j), # this is the point to label
                    textcoords="offset points", # how to position the text
                    xytext=(0,10), # distance from text to points (x,y)
                    ha='center') # horizontal alignment can be left, right or center

    plt.xlabel("x-axis")
    plt.ylabel("y-axis")
    plt.title("The Title")

    plt.show()

def covertDistanceToEuclidWorld(sweep_data, drone_pos):
    ret = []
    sweep_data = np.array(sweep_data)
    for i in range(sweep_data.shape[0]):
        #print(sweep_data[i, 0], sweep_data[i, 1])
        x = drone_pos[0] + sweep_data[i, 1] * math.sin(math.radians(sweep_data[i, 0])) * 0.001
        y = drone_pos[1] + sweep_data[i, 1] * math.cos(math.radians(sweep_data[i, 0])) * 0.001
        ret.append([x, y])
        
    ret = np.array(ret)

    return ret

def covertDistanceToEuclidLocal(sweep_data, drone_pos):
    ret = []
    sweep_data = np.array(sweep_data)
    
    for i in range(sweep_data.shape[0]):
        x = sweep_data[i, 1] * np.sin(np.radians(sweep_data[i, 0])) * 0.001 
        y = sweep_data[i, 1] * np.cos(np.radians(sweep_data[i, 0])) * 0.001
        ret.append([x, y])
        
    ret = np.array(ret)

    return ret

def runMappingAndSave(lidar_data, drone_data, file_name):
    if len(lidar_data.sweep_data_raw) != len(drone_data.drone_position):
        print("Data missing")
        return 0
    else:
        sweeps_glob = None
        for i in range(len(lidar_data.scan_ID)):
            if len(lidar_data.sweep_data_raw[i]) != 0:
                label = drone_data.pose_ID[i]
                sweep_data_glob = covertDistanceToEuclidWorld(lidar_data.sweep_data_raw[i], drone_data.drone_position[i])
                if i == 0:
                    sweeps_glob = sweep_data_glob
                sweeps_glob = np.concatenate((sweeps_glob, sweep_data_glob), axis=0)
        
        sweeps_glob = resampleMapData(sweeps_glob, 5)

        plt.scatter(sweeps_glob[:,0],sweeps_glob[:,1], s=20, edgecolors='none')
        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        plt.title("Map")

        plt.savefig('map.png')

        header = np.array([[int(0), int(sweeps_glob.shape[0])]])
        sweeps_glob = np.concatenate((header, sweeps_glob), axis=0)

        np.savetxt(file_name, sweeps_glob, delimiter=",", fmt='%.4f')
        
        return sweeps_glob
                

def resampleMapData(sweep_data, step):
    x_max, y_max = np.max(sweep_data, axis=0)[0], np.max(sweep_data, axis=0)[1]
    x_min, y_min = np.min(sweep_data, axis=0)[0], np.min(sweep_data, axis=0)[1]
    
    print("data raw size: {}".format(sweep_data.shape[0]))
    sweep_data = sweep_data[::step,:]
    print("data downsampled size: {}".format(sweep_data.shape[0]))

    return sweep_data
        

def drawLIDARPoints(lidar_data, drone_data, data_to_show=None, with_trace=True):
    if len(lidar_data.sweep_data_raw) != len(drone_data.drone_position):
        print("Data missing")
        return 0
    else:
        # draw drone path
        drone_x = drone_data.drone_position[:,0]
        drone_y = drone_data.drone_position[:,1]
        n = drone_data.pose_ID
        plt.style.use('seaborn-pastel')

        if data_to_show is None:
            for i in range(len(lidar_data.scan_ID)):
                if len(lidar_data.sweep_data_raw[i]) != 0:
                    label = drone_data.pose_ID[i]
                    sweep_data_glob = covertDistanceToEuclidWorld(lidar_data.sweep_data_raw[i], drone_data.drone_position[i])
                    
                    pts_x = sweep_data_glob[:,0]
                    pts_y = sweep_data_glob[:,1]

                    plt.annotate(label, # this is the text
                            drone_data.drone_position[i], # this is the point to label
                            textcoords="offset points", # how to position the text
                            xytext=(0,10), # distance from text to points (x,y)
                            ha='center') # horizontal alignment can be left, right or center

                    plt.scatter(pts_x, pts_y, s=20, edgecolors='none')
                    
        else:
            drone_x = []
            drone_y = []
            for id in data_to_show:
                if len(lidar_data.sweep_data_raw[id]) != 0:
                    drone_x.append(drone_data.drone_position[id][0])
                    drone_y.append(drone_data.drone_position[id][1])

                    label = id
                    sweep_data_glob = covertDistanceToEuclidWorld(lidar_data.sweep_data_raw[id], drone_data.drone_position[id])

                    pts_x = sweep_data_glob[:,0]
                    pts_y = sweep_data_glob[:,1]

                    plt.annotate(label, # this is the text
                            drone_data.drone_position[id], # this is the point to label
                            textcoords="offset points", # how to position the text
                            xytext=(0,10), # distance from text to points (x,y)
                            ha='center') # horizontal alignment can be left, right or center

                    plt.scatter(pts_x, pts_y, s=20, edgecolors='none')

        if with_trace:
            plt.plot(drone_x,drone_y)
            plt.scatter(drone_x, drone_y, s=20, edgecolors='none', c='red')

        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        plt.title("Lidar Sweep Data")

        plt.show()

def main():
    helper()
    
    drone_data = DroneTrajectoryData('FlightPath.csv')
    lidar_data = LidarSweepData('LIDARPoints.csv')

    drawDronePath(drone_data)
    drawLIDARPoints(lidar_data, drone_data, data_to_show=None, with_trace=True)
    
    map_data = runMappingAndSave(lidar_data, drone_data, 'map.csv')

    return 0


if __name__ == "__main__":
    main()