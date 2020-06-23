import os
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.animation import FuncAnimation
from data_loader import DataLoader, DroneTrajectoryData, LidarSweepData
from optimizer import RRTSolver
from rrt import RRT
from visualizer import Drawer

show_animation = True

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

def covertDistanceToEuclidWorld(sweep_data, drone_pos):
    ret = []
    sweep_data = np.array(sweep_data)
    for i in range(sweep_data.shape[0]):
        #print(sweep_data[i, 0], sweep_data[i, 1])
        x = drone_pos[0] + sweep_data[i, 1] * math.cos(math.radians(sweep_data[i, 0])) * 0.001
        y = drone_pos[1] + sweep_data[i, 1] * math.sin(math.radians(sweep_data[i, 0])) * -0.001
        ret.append([x, y])
        
    ret = np.array(ret)

    return ret

def covertDistanceToEuclidLocal(sweep_data, drone_pos):
    ret = []
    sweep_data = np.array(sweep_data)
    
    for i in range(sweep_data.shape[0]):
        x = sweep_data[i, 1] * np.cos(np.radians(sweep_data[i, 0])) * 0.001 
        y = sweep_data[i, 1] * np.sin(np.radians(sweep_data[i, 0])) * -0.001
        ret.append([x, y])
        
    ret = np.array(ret)

    return ret

def runMappingAndSave(lidar_data, drone_data, file_name):
    if len(lidar_data.sweep_data_raw) != len(drone_data.drone_position):
        print("Data missing")
        return 0
    else:
        sweeps_glob = None
        step = 3

        for i in range(len(lidar_data.scan_ID)):
            if len(lidar_data.sweep_data_raw[i]) != 0:
                label = drone_data.pose_ID[i]
                sweep_data_glob = covertDistanceToEuclidWorld(lidar_data.sweep_data_raw[i], drone_data.drone_position[i])
                if i == 0:
                    sweeps_glob = sweep_data_glob
                sweeps_glob = np.concatenate((sweeps_glob, sweep_data_glob), axis=0)
        

        sweeps_glob = sweeps_glob[::step,:]

        plt.scatter(sweeps_glob[:,0],sweeps_glob[:,1], s=20, edgecolors='none')
        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        plt.title("Indoor Map")

        plt.savefig('map.png')
        plt.close('all')
        # Assign header and Save
        header = np.array([[int(0), int(sweeps_glob.shape[0])]])
        np.savetxt(file_name, header.astype(int), delimiter=",", fmt='%i')

        # Concat data and Save
        with open(file_name, "ab") as f:
            np.savetxt(f, sweeps_glob, delimiter=",", fmt='%.4f')
        
        return sweeps_glob


def gridlizeMapData(sweep_data):
    x_max, y_max = np.max(sweep_data, axis=0)[0], np.max(sweep_data, axis=0)[1]
    x_min, y_min = np.min(sweep_data, axis=0)[0], np.min(sweep_data, axis=0)[1]
    
    split_ratio = 5
    pad = 20
    tuple_data = []

    mask = np.zeros((int(x_max-x_min)*split_ratio + pad, int(y_max - y_min)*split_ratio + pad), dtype=np.int)
    # print(mask.shape)
    # # print(sweep_data.shape[0])
    # print(int(x_max)*split_ratio)
    # print(int(x_min)*split_ratio)
    # print(int(y_max)*split_ratio)
    # print(int(y_min)*split_ratio)

    for i in range(sweep_data.shape[0]):
        x_grid = int(sweep_data[i][0] * split_ratio ) - (int(x_min)*split_ratio) + int(pad / 2)
        y_grid = int(sweep_data[i][1] * split_ratio ) - (int(y_min)*split_ratio) + int(pad / 2)
        
        if abs(x_grid) > 0 and abs(y_grid) > 0:
            mask[x_grid][y_grid] = 1
            if i % 2 == 0:
                tuple_data.append((sweep_data[i][0], sweep_data[i][1], 0.1))
    print("tuple size: {}".format(len(tuple_data)))
    return mask, tuple_data

def runRerouting(map_grid, obstacle_list, start_grid, end_grid):
    startpos = (start_grid[0], start_grid[1])
    endpos = (end_grid[0], end_grid[1])
    obstacles = obstacle_list

    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(start=[start_grid[0], start_grid[1]],
              goal=[end_grid[0], end_grid[1]],
              rand_area=[-2, 15],
              obstacle_list=obstacle_list)

    # rrt = RRT(start=[0, 0],
    #           goal=[6., 10.],
    #           rand_area=[-2, 15],
    #           obstacle_list=obstacleList)

    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        print(path)
        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
    # startpos = (0., 0.)
    # endpos = (5., 5.)
    # obstacles = [(1., 1.), (2., 2.)]
    
    # n_iter = 200
    # radius = 0.1
    # stepSize = 1.0

    # plt.close('all')
    # plt.xlabel("x-axis")
    # plt.ylabel("y-axis")
    # plt.title("Grid Map")
    # plt.imshow(map_grid, origin='lower', aspect='auto')
    # plt.savefig('grid_path.png')

    return path

def main():
    helper()
    
    drone_data = DroneTrajectoryData('FlightPath.csv')
    lidar_data = LidarSweepData('LIDARPoints.csv')
    # 1. Visualizing Task
    drawer = Drawer()
    #drawer.drawDronePath(drone_data)
    #drawer.drawLIDARPoints(lidar_data, drone_data, data_to_show=None, with_trace=True)
    
    # 2. Mapping Task
    map_data = runMappingAndSave(lidar_data, drone_data, 'map.csv')
    map_grid, obstacle_list = gridlizeMapData(map_data)
    
    # 3. Path Optimization Task
    start_grid = drone_data.drone_position[0]
    end_grid = drone_data.drone_position[-2]
    runRerouting(map_grid, obstacle_list, start_grid, end_grid)

    return 0


if __name__ == "__main__":
    main()