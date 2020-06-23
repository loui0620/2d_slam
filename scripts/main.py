import sys, getopt
import os
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.animation import FuncAnimation
from data_loader import DataLoader, DroneTrajectoryData, LidarSweepData
from optimizer import RRTSolver
from dijkstra import Dijkstra
from visualizer import Drawer
from mapping import Mapping

show_animation = True
save_result = False
save_path = os.path.dirname(os.path.abspath(__file__)) + "/../result/"

def helper():
    print("====================================================================")
    print("This is assignment from Socville. Which contains functions included:\n\
    1. Visualizing Drone Lidar Data .\n\
    2. Mapping Lidar Data & save result.\n\
    3. Path Reroouting. & save result")
    print("====================================================================")
    print("Augments:")
    print("Assign drone data: --drone_data <drone_data_path>")
    print("Assign lidar data: --lidar_data <lidar_data_path>")
    print("Run visualizer: --run_visualize True")
    print("Set Sweep ID to Visualizer: --data_to_show <scan_ID_list> i.e. [0,1,2,3]")
    print("Save Result: --save_result")
    print("\n")
    print ('Usage Example: python scripts/main.py --path FlightPath.csv --lidar_data LIDARPoints.csv -n [0,1,2] --save true\n')


def ask_user():
    helper()
    print("\nChoose Function to Run: (A) visualization (B) mapping (C) rerouting (D) all")
    response = ''
    while response.lower() not in {"a", "b", "c", "d"}:
        response = input("Please enter above options: ")
    return response.lower()


def runRerouting(obstacle_list, start_grid, end_grid, is_save=False):
    # start and goal position
    sx = start_grid[0]  # [m]
    sy = start_grid[1]  # [m]
    gx = end_grid[0]  # [m]
    gy = end_grid[1]  # [m]
    grid_size = 0.4  # [m]
    robot_radius = 0.3  # [m]

    # set obstacle positions
    ox, oy = [], []
    path = []

    for i in range(len(obstacle_list)):
        ox.append(obstacle_list[i][0])
        oy.append(obstacle_list[i][1])

    if show_animation:  # pragma: no cover
        plt.title("Rerouting")
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    for i in range(len(rx)):
        path.append([i, 1])
        path.append([rx[i], ry[i]])

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        if is_save:
            fig = plt.gcf()
            fig.savefig(save_path + 'task3_rerouted.png')
        plt.pause(0.01)
        plt.show()
        
    return path


def main(argv):
    path_file = ''
    lidar_file = ''
    data_to_show = []
    
    try:
        opts, args = getopt.getopt(argv,"hp:d:n:s:",["path=","lidar_data=","show_data=", "save="])
    except getopt.GetoptError:
        print ('\nUsage Example: python scripts/main.py --path FlightPath.csv --lidar_data LIDARPoints.csv -n [0,1,2] --save true\n')
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            helper()
            sys.exit()
        elif opt in ("-p", "--path"):
            path_file = arg
        elif opt in ("-d", "--lidar_data"):
            lidar_file = arg
        elif opt in ("-n", "--show_data"):
            list_str = arg
            if list_str != '[]':
                data_to_show = list(map(int, list_str.strip('[]').split(',')))
        elif opt in ("-s", "--save"):
            is_save = arg
            if is_save.lower() == 'true':
                save_result = True
            else:
                save_result = False
        else:
            print ('\nUsage Example: python scripts/main.py --path FlightPath.csv --lidar_data LIDARPoints.csv -n [0,1,2] --save true\n')
            sys.exit()

    drone_data = DroneTrajectoryData(path_file)
    lidar_data = LidarSweepData(lidar_file)
    
    choice = ask_user()

    route_path = save_path + 'ReroutedPath.csv'
    map_path = save_path + 'Map.csv'

    # 1. Visualizing Task
    if choice in {'a', 'd'}:
        drawer = Drawer()
        drawer.save_result = save_result
        drawer.drawDronePath(drone_data)
        drawer.drawLIDARPoints(lidar_data, drone_data, data_to_show=data_to_show, with_trace=True)
    
    # 2. Mapping Task
    if choice in {'b', 'd'}:
        map_mgr = Mapping()
        map_mgr.save_result = save_result    
        map_data = map_mgr.runMappingAndSave(lidar_data, drone_data, map_path)
        map_grid, obstacle_list = map_mgr.gridlizeMapData(map_data)
        
    # 3. Path Optimization Task
    if choice in {'c', 'd'}: 
        if choice == 'c':
            map_mgr = Mapping()  
            map_mgr.save_result = save_result    
            map_data = map_mgr.runMappingAndSave(lidar_data, drone_data, map_path)
            map_grid, obstacle_list = map_mgr.gridlizeMapData(map_data)

        start_grid = drone_data.drone_position[0] # 0-th 
        end_grid = drone_data.drone_position[-2] # 16-th 
        #end_grid = [20,4] 

        path = runRerouting(obstacle_list, start_grid, end_grid, save_result)
        
        with open(route_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            cnt = 0
            for i in range(len(path)):
                if (i % 2) == 0:
                    writer.writerow([int(i), int(1)])
                else:
                    writer.writerow([path[i][0], path[i][1]])

    print("program finished.")

    return 0


if __name__ == "__main__":
    main(sys.argv[1:])