from viconnexusapi import ViconNexus
import os
import numpy as np
import matplotlib.pyplot as plt
import heapq

vicon = ViconNexus.ViconNexus()
player_file = "Play-30"
    # for file in os.path(f"C:/Users/ahumphreys/EXOS_Processing/{player_file}"):
    #     vicon.OpenTrial(file, 30)

file = fr"C:\Users\ahumphreys\EXOS_Processing\{player_file}\Cleat01\{player_file.replace('-', '')}_Cleat01_Trial14"
vicon.OpenTrial(file, 30)

subject = vicon.GetSubjectNames()[0]

user_defined_region = vicon.GetTrialRegionOfInterest()

foot_contact_markers = []

right_foot_markers = (
    'RD2P',
    'RD5P',
    'RHEE',
)

left_foot_markers = (
    'LD5P',
    # 'LHEE'
)

markers = left_foot_markers + right_foot_markers

z_coords = {marker: [] for marker in markers}
z_velo = {marker: [] for marker in markers}
z_accel = {marker: [] for marker in markers}

for marker in markers:
    z_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[2] for frame in range(user_defined_region[0], user_defined_region[1]))

for marker in markers:   
    z_velo[marker].extend(np.gradient(z_coords[marker]))
for marker in markers:   
    z_accel[marker].extend(np.diff(z_coords[marker], 2))

def find_cycles(marker: str = 'RD2P'):
    foot_down_frames = []
    def is_z_accel_peak(i, threshold: float = 4):
        return marker_z_accel[i] > threshold and marker_z_accel[i-1] < marker_z_accel[i] > marker_z_accel[i+1]
 
    def is_z_velo_trough(i, threshold: float = -4):
        return marker_z_velo[i] < threshold and marker_z_velo[i-1] > marker_z_velo[i] < marker_z_velo[i+1]
    
    z_accel_peak = z_velo_trough = False
    marker_z_accel = z_accel[marker]
    marker_z_velo = z_velo[marker]
 
    for i in range(1, len(marker_z_accel) - 1):
        if is_z_accel_peak(i):
            z_accel_peak = True
        
        if is_z_velo_trough(i):
            z_velo_trough = True

        if z_accel_peak and z_velo_trough and marker_z_accel[i-1] > marker_z_velo[i-1] and marker_z_accel[i] < marker_z_velo[i]:
            foot_down_frames.append(i + user_defined_region[0])
            z_accel_peak = z_velo_trough = False

            
    return foot_down_frames

def plot(markers):
    plt.figure(figsize=(10,6))

    for marker in markers:
        frames = np.arange(user_defined_region[0], user_defined_region[1])
        # plt.plot(frames, x_coords[marker])
        plt.plot(frames, z_coords[marker])
        # plt.plot(frames, z_velo[marker], label = marker + ' z_velo')
        frames = np.arange(user_defined_region[0], user_defined_region[1] - 2)
        plt.plot(frames, z_accel[marker], label = marker + ' z accel')

    plt.xlabel('Frame')
    plt.title('Kinematics over time')
    plt.legend()
    plt.grid(True)
    plt.show()

def calculate_cycles(list1, list2, list3):
    points = []
    heap = []
    final_points = []

    heapq.heappush(heap, (list1[0], 0, list1))
    heapq.heappush(heap, (list2[0], 0, list2))
    heapq.heappush(heap, (list3[0], 0, list3))

    while heap:
        value, idx, arr = heapq.heappop(heap)
        points.append(value)
        if idx + 1 < len(arr):
            heapq.heappush(heap, (arr[idx + 1], idx + 1, arr))
        
    print(points)
    diff = 0
    in_bounds = False
    for i in range(len(points)-1):
        diff = abs(points[i] - points[i+1])
        if diff <= 15 and not in_bounds:
            final_points.append(points[i])
            in_bounds = True
        elif diff > 15:
            in_bounds = False
        


    return final_points
            

    


def main():
    print(calculate_cycles(find_cycles(right_foot_markers[0]), find_cycles(right_foot_markers[1]), find_cycles(right_foot_markers[2])))
    plot(right_foot_markers)

if __name__ == "__main__":
    main()


#vicon.SaveTrial()