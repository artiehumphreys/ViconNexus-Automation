import heapq
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from vicon import Vicon

right_foot_markers = (
    'RD2P',
    'RD5P',
    'RHEE',
    'RLATH',
    'RD5M',
    'RD1M',
    'RP1M',
)

left_foot_markers = (
    'LD2P',
    'LD5P',
    'LHEE',
    'LLATH',
    'LD5M',
    'LD1M',
    'LP1M',
)

class Marker:
    def __init__(self, name: str, vicon: Vicon):
        self.marker = name
        self.z_coords = []
        self.y_coords = []
        self.x_coords = []
        self.z_velo = []
        self.z_accel = []
        self.z_jerk = []
        self.vicon = vicon

        trajectory = vicon.fetch_trajectory(self.marker)
        for pos in trajectory:
            self.z_coords.append(pos[2])
            self.y_coords.append(pos[1])
            self.x_coords.append(pos[0])
        if self.z_coords:
            self.z_velo.append(np.gradient(self.z_coords)) 
            self.z_accel.append(np.diff(self.z_coords, 2))
            self.z_accel.append(np.diff(self.z_coords, 3))
        
        self.z_velo = self.z_velo[0]
        self.z_accel = self.z_accel[0]

    def is_z_accel_peak(self, i, threshold: float = 4):
        return self.z_accel[i] > threshold and self.z_accel[i-1] < self.z_accel[i] > self.z_accel[i+1]
    
    def is_z_velo_trough(self, i, threshold: float = -4):
        return self.z_velo[i] < threshold and self.z_velo[i-1] > self.z_velo[i] < self.z_velo[i+1]

    def is_velo_peak(self, i, threshold: float = 4):
            return self.z_velo > threshold
    
    def is_jerk_trough(self, i, threshold: float = -0.75):
            return self.z_jerk[i] < threshold and self.z_jerk[i-1] > self.z_jerk[i] < self.z_jerk[i+1]

    def find_foot_strike(self) -> list:
        lower_frame_bound = self.vicon.get_region_of_interest()[0]
        foot_down_frames = []
        
        z_accel_peak = False
        z_velo_trough = False
    
        for i in range(1, len(self.z_accel) - 1):
            if self.is_z_accel_peak(i):
                z_accel_peak = True    

            if self.is_z_velo_trough(i):
                z_velo_trough = True

            if z_accel_peak and z_velo_trough and self.z_accel[i-1] > self.z_velo[i-1] and self.z_accel[i] < self.z_velo[i] and self.z_coords[i] < 120:
                foot_down_frames.append(i + lower_frame_bound)
                z_accel_peak = z_velo_trough = False

        return foot_down_frames

    def find_foot_up(self, foot_down_frames):
        lower_frame_bound = self.vicon.get_region_of_interest()[0]
        foot_up_frames = []

        z_velo_peak = False
        z_velo_trough = False
        z_jerk_trough = False

        
        
        for i in range(foot_down_frames[0], len(self.z_accel) - 1):
            if self.is_velo_peak(i):
                z_velo_peak = True

            if self.is_jerk_trough(i):
                z_jerk_trough = True

            if self.is_velo_trough(i, -2.5):
                z_velo_trough = True

            if z_velo_peak and jerk_trough:
                continue

    def find_frames_from_data(self, list1: list, list2: list, list3: list) -> list:
        points = []
        heap = []
        final_points = []
        if list1:
            heapq.heappush(heap, (list1[0], 0, list1))
        if list2:
            heapq.heappush(heap, (list2[0], 0, list2))
        if list3:
            heapq.heappush(heap, (list3[0], 0, list3))

        while heap:
            value, idx, arr = heapq.heappop(heap)
            points.append(value)
            if idx + 1 < len(arr):
                heapq.heappush(heap, (arr[idx + 1], idx + 1, arr))

        diff = 0
        in_bounds = False
        for i in range(len(points)-1):
            diff = abs(points[i] - points[i+1])
            if diff <= 19 and not in_bounds:
                final_points.append(points[i])
                in_bounds = True
            elif diff > 19:
                in_bounds = False

        return final_points

    def plot_markers(self, lower_bound: int, upper_bound: int):
        plt.figure(figsize=(10,6))

        frames = np.arange(lower_bound, upper_bound)
        plt.plot(frames, self.z_coords, label = self.marker + ' z coord')
        plt.plot(frames, self.z_velo, label = self.marker + ' z velo')
        frames = np.arange(lower_bound, upper_bound - 2)
        plt.plot(frames, self.z_accel, label = self.marker + ' z accel')

        plt.xlabel('Frame')
        plt.title('Kinematics over time')
        plt.legend()
        plt.grid(True)
        plt.show()

def main():
    vicon = Vicon()
    lower, upper = vicon.get_region_of_interest()
    marker = Marker('RD2P', vicon)
    marker.plot_markers(lower, upper)
    lis = []
    for marker in right_foot_markers[:3]:
        marker = Marker(marker, vicon)
        lis.append(marker.find_foot_strike())
        print(lis)
        
    print(marker.find_frames_from_data(lis[0], lis[1], lis[2]))

if __name__ == "__main__":
    main()