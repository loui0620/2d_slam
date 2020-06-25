import csv
import os
import numpy as np
import matplotlib as pyplot

class DataLoader():
    def __init__(self):
        self.path_ID = []
        self.drone_pos = []
        self.scan_ID = []
        self.sweep_data = []
        self.dic_sweep_data = {}

    def readFlightPath(self, file_path):
        with open(file_path, newline='') as csvfile:
            rows = csv.reader(csvfile, delimiter=':')
            read_cnt = 0

            for row in rows:
                if row :
                    if read_cnt % 2 == 0:
                        temp1 = int(row[0].strip().split(',')[0])
                        # temp2 = int(row[0].strip().split(',')[1])   
                        self.path_ID.append(temp1)
                    else:
                        temp1 = float(row[0].strip().split(',')[0])
                        temp2 = float(row[0].strip().split(',')[1])   
                        self.drone_pos.append([temp1, temp2])  

                read_cnt += 1
        
        return np.array(self.path_ID), np.array(self.drone_pos)

    def readLidarData(self, file_path):
        with open(file_path, newline='') as csvfile:
            rows = csv.reader(csvfile, delimiter=':')
            
            read_cnt = 0
            set_cnt = 0
            
            head_idx = 0
            data_length = 0

            read_open = False
            sweep_temp = []

            for row in rows:
                
                temp1 = row[0].strip().split(',')[0]
                temp2 = row[0].strip().split(',')[1]
                
                if self.isRepresentInt(temp1):
                    if int(temp1) == set_cnt:
                        self.scan_ID.append(int(temp1))
                        head_idx = read_cnt
                        data_length = int(temp2)
                        
                        sweep_temp = []
                        set_cnt += 1
                        
                    
                else:
                    if read_cnt <=  (head_idx + data_length):
                        sweep_temp.append([float(temp1), float(temp2)])

                        if read_cnt == head_idx + data_length:
                            self.sweep_data.append(sweep_temp)


                read_cnt += 1

        return np.array(self.scan_ID), np.array(self.sweep_data)
    
    def isRepresentInt(self, s):
        try: 
            int(s)
            return True
        except ValueError:
            return False


class DroneTrajectoryData(DataLoader):
    def __init__(self, file_path):
        self.pose_ID = None
        self.drone_position = None
        self.loader = DataLoader()
        self.pose_ID, self.drone_position = self.loader.readFlightPath(file_path)


class LidarSweepData(DataLoader):
    def __init__(self, file_path):
        self.scan_ID = None
        self.sweep_data_raw = None
        self.loader = DataLoader()
        self.scan_ID, self.sweep_data_raw = self.loader.readLidarData(file_path)

