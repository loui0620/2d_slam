# Scoville Challenge

- Input: (1) **FlightPath.csv** in (x, y), which contains drone position in XY coordinate. (2) **LIDARPoints.csv** lidar data sweeped in each position on trajectory.
- This program will output 
  - task1: visualized window of trajectory & LIDARPoints file.
  - task2: LIDAR data mapping to get room dimensions. Saved as ```map.csv```
  - task3: Optimized rerouting of existed start & end position. Saved as ```ReroutedPath.csv```
  - By input argument, visualized images will be saved in ```/result``` folder.
- This program is implemented in Python.
  

**Program Prerequisite:**
  - Language: Python
  - Platform: Ubuntu
  - Open terminal and install requiring package, ```csv```, ```numpy``` and ```matplotlib``` by following commands:
```bash
$ python -m pip install python-csv
$ python -m pip install numpy
$ python -m pip install matplotlib
```

**Arguments Input And Run Program:**
  - Open terminal in project root folder. Run program as following command: ```python -m script/main.py -h```
  - Arguments List:
      - --path: File path of **FlightPath.csv**
      - --lidar_data: File path of **LIDARPOINTS.csv**
      - --n: Positions choose to display: *i.e. [0,1,2,3]*
      - --save: Save image in ```/result``` folder
  - You could enter either **file name** or **absolute file path** and X to run program.
  - **Example Usage**
      - ```python scripts/main.py --path FlightPath.csv --lidar_data LIDARPoints.csv -n [0,1,2,3,4] --save true```
  - Once program runs, you need to choose tasks to run as follow:
```bash
Choose Function to Run: (A) visualization (B) mapping (C) rerouting (D) all 
Please enter above options:
```
Choose by typing Alphbet header, press ```d``` to run all functions.


# Detail and Analysis

  - **Task1: Visualizing**
    - Read from *FlightPath.csv* and *LIDARPoints* to display trajectory and maps. Could display desired LIDAR sweeps separately by input argument ```-n [1,3,5...etc]```
    - ![alt text](https://i.imgur.com/HS76e2R.png)
  - **Task2: Mapping**
    - By given position and identified sweep data, this task is aimed to build full map and store data in ```result/Map.csv``` at root folder. The format is same as LIDARPoint data, first line is (map_ID, data_line_num), and following are the X&Y data points.
    - Generated 2D points will be converted into 2D binary grid mask to provide data for the re-routing task coming next.
    ![alt text](https://i.imgur.com/VFTceKM.png)
  -  **Task3: Re-routing**
        - This function has default input of start & end point from *FlightPath.csv*. It's also optional to assign other different goal within area, as figure shows below.
        ![alt text](https://i.imgur.com/Msq17gU.png)
        -  I chose *Dijkstra algorithm* to run path planning this time. At first, I chose another familiar method which is *RRT(Rapid-exploring Random Tree)* to run it. RRT is a useful method to solve multi-DOF robot planning such as multi-joint planning. But due to the approach, RRT is a pure random searching method, in-sensitivity to environment feature and drastically-affected by narrow-tunneled feature between rooms. RRT could barely achieve convergence before reaching the search time limit, as figure shows above. 
        -  Then replace it by Dijkstra method, Dijkstra has some advantages such as finds shortest path in O( E+ V Log(V) ) if using a min priority queue. And assurance of shortest-path retrieval. But it fails in cases where you have a negative edge inside search graph. By setting search step and grid size we could assure it could find shortest path even passing through the narrow tunnels.
        -  This function will output as ```result/ReroutedPath.csv``` in root folder. The output format is same as FlightPath data, first line is (position_ID, 1), and the following X&Y data points.

# Reference Materials
  - [Dijkstra](https://www.programiz.com/dsa/dijkstra-algorithm)
  - [RRT](https://medium.com/@theclassytim/robotic-path-planning-rrt-and-rrt-212319121378)
  - [Python Robotics](https://github.com/AtsushiSakai/PythonRobotics)
