import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_loader import DataLoader, DroneTrajectoryData, LidarSweepData
import math

save_path = os.path.dirname(os.path.abspath(__file__)) + "/../result/"

class Drawer():
    def __init__(self):
        self.save_result = False

    def drawDronePath(self, data):
        x = data.drone_position[:,0]
        y = data.drone_position[:,1]
        n = data.pose_ID

        plt.style.use('seaborn-pastel')
        
        plt.plot(x,y)
        plt.scatter(x, y, s=20, edgecolors='none', c='red')

        for i,j,k in zip(x,y,n):
            label = k
            plt.annotate(label, 
                        (i,j),
                        textcoords="offset points",
                        xytext=(0,10),
                        ha='center')

        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        plt.title("Drone Trajectory")
        if self.save_result:
            plt.savefig(save_path + 'task1_trajectory.png')
        plt.show()


    def drawLIDARPoints(self, lidar_data, drone_data, data_to_show=None, with_trace=True):
        if len(lidar_data.sweep_data_raw) != len(drone_data.drone_position):
            print("Data missing")
            return 0
        else:
            # draw drone path
            drone_x = drone_data.drone_position[:,0]
            drone_y = drone_data.drone_position[:,1]
            n = drone_data.pose_ID
            plt.style.use('seaborn-pastel')

            if len(data_to_show) == 0:
                for i in range(len(lidar_data.scan_ID)):
                    if len(lidar_data.sweep_data_raw[i]) != 0:
                        label = drone_data.pose_ID[i]
                        sweep_data_glob = self.covertDistanceToEuclidWorld(lidar_data.sweep_data_raw[i], drone_data.drone_position[i])
                        
                        pts_x = sweep_data_glob[:,0]
                        pts_y = sweep_data_glob[:,1]

                        plt.annotate(label, 
                                drone_data.drone_position[i], 
                                textcoords="offset points", 
                                xytext=(0,10), 
                                ha='center') 

                        plt.scatter(pts_x, pts_y, s=20, edgecolors='none')
                        
            else:
                drone_x = []
                drone_y = []
                for id in data_to_show:
                    if len(lidar_data.sweep_data_raw[id]) != 0:
                        drone_x.append(drone_data.drone_position[id][0])
                        drone_y.append(drone_data.drone_position[id][1])
                        
                        label = id
                        sweep_data_glob = self.covertDistanceToEuclidWorld(lidar_data.sweep_data_raw[id], drone_data.drone_position[id])

                        pts_x = sweep_data_glob[:,0]
                        pts_y = sweep_data_glob[:,1]

                        plt.annotate(label, 
                                drone_data.drone_position[id], 
                                textcoords="offset points", 
                                xytext=(0,10),
                                ha='center') 

                        plt.scatter(pts_x, pts_y, s=20, edgecolors='none')

            if with_trace:
                plt.plot(drone_x,drone_y)
                plt.scatter(drone_x, drone_y, s=20, edgecolors='none', c='red')

            plt.xlabel("x-axis")
            plt.ylabel("y-axis")
            plt.title("Lidar Sweep Data")
            if self.save_result:
                plt.savefig(save_path + 'task1_visualized.png')
            plt.show()
        
    def covertDistanceToEuclidWorld(self, sweep_data, drone_pos):
        ret = []
        sweep_data = np.array(sweep_data)
        for i in range(sweep_data.shape[0]):
            #print(sweep_data[i, 0], sweep_data[i, 1])
            x = drone_pos[0] + sweep_data[i, 1] * math.cos(math.radians(sweep_data[i, 0])) * 0.001
            y = drone_pos[1] + sweep_data[i, 1] * math.sin(math.radians(sweep_data[i, 0])) * -0.001
            ret.append([x, y])
            
        ret = np.array(ret)

        return ret