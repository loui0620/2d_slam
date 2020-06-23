# Socville Challenge: 2D SLAM

- Input: (1) FlightPath.csv in (x, y), which contains drone position in XY coordinate. (2) lidar data sweeped in each position on trajectory.
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
    - By given position and identified sweep data, this task is aimed to build full map and store data in ```map.csv``` at root folder.
    - Generated 2D points will be converted into 2D binary grid image to provide data for the re-routing task coming next.
    ![alt text](https://i.imgur.com/VFTceKM.png)
  -  **Task3: Re-routing**
        - This function has default input of start & end point from *FlightPath.csv*. It's also optional to assign other different goal within area, as figure shows below.
        ![alt text](https://i.imgur.com/Msq17gU.png)
        -  I chose *Dijkstra algorithm* to run path planning this time. At first I used familiar RRTStar method to run it.
