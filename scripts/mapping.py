import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from data_loader import DataLoader, DroneTrajectoryData, LidarSweepData
import math

save_path = os.path.dirname(os.path.abspath(__file__)) + "/../result/"

class Mapping():
    def __init__(self):
        self.save_result = False

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

    def covertDistanceToEuclidLocal(self, sweep_data, drone_pos):
        ret = []
        sweep_data = np.array(sweep_data)
        
        for i in range(sweep_data.shape[0]):
            x = sweep_data[i, 1] * np.cos(np.radians(sweep_data[i, 0])) * 0.001 
            y = sweep_data[i, 1] * np.sin(np.radians(sweep_data[i, 0])) * -0.001
            ret.append([x, y])
            
        ret = np.array(ret)

        return ret

    def runMappingAndSave(self, lidar_data, drone_data, file_name):
        if len(lidar_data.sweep_data_raw) != len(drone_data.drone_position):
            print("Data missing")
            return 0
        else:
            sweeps_glob = None
            step = 3

            for i in range(len(lidar_data.scan_ID)):
                if len(lidar_data.sweep_data_raw[i]) != 0:
                    label = drone_data.pose_ID[i]
                    sweep_data_glob = self.covertDistanceToEuclidWorld(lidar_data.sweep_data_raw[i], drone_data.drone_position[i])
                    if i == 0:
                        sweeps_glob = sweep_data_glob
                    sweeps_glob = np.concatenate((sweeps_glob, sweep_data_glob), axis=0)
            

            sweeps_glob = sweeps_glob[::step,:]

            plt.scatter(sweeps_glob[:,0],sweeps_glob[:,1], s=20, edgecolors='none')
            plt.xlabel("x-axis")
            plt.ylabel("y-axis")
            plt.title("Indoor Map")
            
            if self.save_result:
                print("result saved")
                plt.savefig(save_path + 'task2_point_map.png')
            
            plt.close('all')
            # Assign header and Save
            header = np.array([[int(0), int(sweeps_glob.shape[0])]])
            np.savetxt(file_name, header.astype(int), delimiter=",", fmt='%i')

            # Concat data and Save
            with open(file_name, "ab") as f:
                np.savetxt(f, sweeps_glob, delimiter=",", fmt='%.4f')
            
            return sweeps_glob


    def gridlizeMapData(self, sweep_data):
        x_max, y_max = np.max(sweep_data, axis=0)[0], np.max(sweep_data, axis=0)[1]
        x_min, y_min = np.min(sweep_data, axis=0)[0], np.min(sweep_data, axis=0)[1]
        
        split_ratio = 5
        pad = 20
        tuple_data = []

        mask = np.zeros((int(x_max-x_min)*split_ratio + pad, int(y_max - y_min)*split_ratio + pad), dtype=np.int)

        for i in range(sweep_data.shape[0]):
            x_grid = int(sweep_data[i][0] * split_ratio ) - (int(x_min)*split_ratio) + int(pad / 2)
            y_grid = int(sweep_data[i][1] * split_ratio ) - (int(y_min)*split_ratio) + int(pad / 2)
            
            if abs(x_grid) > 0 and abs(y_grid) > 0:
                mask[x_grid][y_grid] = 1
                if i % 4 == 0:
                    tuple_data.append([sweep_data[i][0], sweep_data[i][1]])

        plt.close('all')
        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        plt.title("Grid Map")
        plt.imshow(mask.transpose(), origin='lower', aspect='auto')
        if self.save_result:
            plt.savefig(save_path + 'task2_grid_map.png')
        plt.close('all')
        
        return mask, tuple_data