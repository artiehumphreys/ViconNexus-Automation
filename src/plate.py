import numpy as np
import matplotlib.pyplot as plt
import math
from vicon import Vicon
from foot import Foot

plate_configs = {
    (2712.0,300.0,0.0) : 'Plate1',
    (2712.0,903.0,0.0) : 'Plate2', 
    (2109.0,300.0,0.0) : 'Plate3', 
    (2109.0,903.0,0.0) : 'Plate4',
    (1506.0,300.0,0.0) : 'Plate5',
    (1506.0,903.0,0.0) : 'Plate6',
    (903.0,300.0,0.0) : 'Plate7',
    (903.0,903.0,0.0) : 'Plate8',
    (300.0,300.0,0.0) : 'Plate9',
}

class Plate:
    def __init__ (self, name):
        self.name = name
        self.vicon = Vicon()
        self.wt = ()
        self.fx = []
        self.fy = []
        self.fz = []
        self.dx = []
        self.dy = []
        self.dz = []
        self.copx = []
        self.copy = []
        self.copz = []
        self.mx = []
        self.my = []
        self.mz = []
        self.fetch_plate_data()

    def __str__(self):
        return self.name

    def calculate_gradient(self, dimension = 'x'):
        if self.fx and self.fy and self.fz:
            match dimension:
                case 'x':
                    self.dx = np.gradient(self.fx)
                case 'y':
                    self.dy = np.gradient(self.fy)
                case 'z':
                    self.dz = np.gradient(self.fz)

    def fetch_plate_data(self):
        deviceIDs = self.vicon.vicon.GetDeviceIDs()
        for deviceID in deviceIDs:
            details = self.vicon.vicon.GetDeviceDetails(deviceID)
            if details[1] == 'ForcePlate':
                wt = details[4].WorldT
                plate_name = plate_configs.get(tuple(wt)) # type: ignore
                if plate_name == self.name:
                    self.wt = wt
                    self.name = plate_name
                    self.wr = details[4].WorldR
                    self.fx = self.vicon.vicon.GetDeviceChannel(deviceID, 1, 1)[0]
                    self.fy = self.vicon.vicon.GetDeviceChannel(deviceID, 1, 2)[0]
                    self.fz = self.vicon.vicon.GetDeviceChannel(deviceID, 1, 3)[0]
                    self.copx = self.vicon.vicon.GetDeviceChannel(deviceID, 3, 1)[0]
                    self.copy = self.vicon.vicon.GetDeviceChannel(deviceID, 3, 2)[0]
                    self.copz = self.vicon.vicon.GetDeviceChannel(deviceID, 3, 3)[0]
                    self.mx = self.vicon.vicon.GetDeviceChannel(deviceID, 2, 1)[0]
                    self.my = self.vicon.vicon.GetDeviceChannel(deviceID, 2, 2)[0]
                    self.mz = self.vicon.vicon.GetDeviceChannel(deviceID, 2, 3)[0]
                    for x in range(len(self.copx)):
                        a = self.copx[x]
                        b = self.copy[x]
                        self.copx[x] = self.wt[0] - b
                        self.copy[x] = self.wt[1] + a
                    strike = self.find_plate_strikes()
                    return strike

    def find_plate_strikes(self):
        lower, upper = self.vicon.get_region_of_interest()
        strike_intervals = []
        possible_strike = False
        fc = 0
        start_frame = end_frame = None
        # 2000hz refresh for plates, 200hz for camera
        for i in range(lower * 10, upper * 10):
            resultant =  math.sqrt(self.fx[i] ** 2 + self.fy[i] ** 2 + self.fz[i] ** 2)
            if resultant > 15:
                if not possible_strike:
                    possible_strike = True
                    start_frame = i
                elif fc > 15:
                    end_frame = i
                fc += 1
            else:
                if start_frame and end_frame:
                    strike_intervals.append((start_frame, end_frame))
                start_frame = None
                end_frame = None
                fc = 0
                possible_strike = False
        return strike_intervals

    def find_plate_matches(self, strike_intervals):
        def is_intersecting(box1, box2):
            min_x1, max_x1, min_y1, max_y1 = box1
            min_x2, max_x2, min_y2, max_y2 = box2
            x_overlap = not(max_x1 < min_x2 or max_x2 < min_x1)
            y_overlap = not(max_y1 < min_y2 or max_y2 < min_y1)
            return x_overlap and y_overlap
    
        def frame_in_strike_interval(j, strike_intervals):
            for intervals in strike_intervals:
                start = intervals[0] // 10
                end = intervals[1] // 10 
                if (start <= j <= end):
                    return True
            return False

        left_foot = Foot('left')
        right_foot = Foot('right')
        results = {'left':[], 'right':[]}
        plate_bounds = [self.wt[0] - 300, self.wt[0] + 300, self.wt[1] - 300, self.wt[1] + 300] # type: ignore
        strike_events = {'right': self.vicon.vicon.GetEvents(self.vicon.subject, 'Right', 'Foot Strike')[0], 'left': self.vicon.vicon.GetEvents(self.vicon.subject, 'Left', 'Foot Strike')[0]}
        off_events = {'right': self.vicon.vicon.GetEvents(self.vicon.subject, 'Right', 'Foot Off')[0], 'left': self.vicon.vicon.GetEvents(self.vicon.subject, 'Left', 'Foot Off')[0]}
        for foot in strike_events:
            for i in range(len(off_events[foot])):
                for j in range(strike_events[foot][i], off_events[foot][i] + 1):
                    bbox = right_foot.calculate_bounding_box(j - self.vicon.get_region_of_interest()[0]) if foot == 'right' else left_foot.calculate_bounding_box(j - self.vicon.get_region_of_interest()[0])
                    min_x, max_x, min_y, max_y = bbox
                    # foot not in bounds of the plates
                    if (2712 < min_x and 300 > max_x and 903 < min_y and 0 > max_y):
                        continue
                    if is_intersecting(bbox, plate_bounds) and frame_in_strike_interval(j, strike_intervals):
                        results[foot].append(j)
        results = self.format_results(results)
        return results
    
    def format_results(self, results):
        left_intervals = []
        right_intervals = []
        left = results['left']
        right = results['right']
        if left:
            start = left[0]
            end = left[0]
            for i in range (1, len(left)):
                if left[i] == end + 1:
                    end = left[i]
                else:
                    left_intervals.append((start, end))
                    start = left[i]
                    end = left[i]
            left_intervals.append((start, end)) 
            results['left'] = left_intervals
        if right:
            start = right[0]
            end = right[0]
            for i in range (1, len(right)):
                if right[i] == end + 1:
                    end = right[i]
                else:
                    right_intervals.append((start, end))
                    start = right[i]
                    end = right[i]
            right_intervals.append((start, end)) 
            results['right'] = right_intervals
        return results

    def plot_forces(self):
        plt.figure(figsize=(10,6))
        frames = np.arange(0, len(self.fx)) 
        self.calculate_gradient('x')
        self.calculate_gradient('y')
        self.calculate_gradient('z')
        plt.plot(frames, self.fx, label = 'x force') 
        plt.plot(frames, self.fy, label = 'y force') 
        plt.plot(frames, self.fz, label = 'z force') 
        plt.plot(frames, self.dx, label = 'dx') 
        plt.plot(frames, self.dy, label = 'dy') 
        plt.plot(frames, self.dz, label = 'dz') 

        plt.xlabel('Frame')
        plt.title('Force over time')
        plt.legend()
        plt.grid(True)
        plt.show()

def driver():
    plates = ['Plate1', 'Plate2', 'Plate3', 'Plate4', 'Plate5', 'Plate6', 'Plate7', 'Plate8', 'Plate9']
    results = {plate: {} for plate in plates}
    for plate in plates:
        print(plate)
        p = Plate(plate)
        intervals = p.fetch_plate_data()
        results[plate] = p.find_plate_matches(intervals)
    return results

if __name__ == "__main__":
    driver()