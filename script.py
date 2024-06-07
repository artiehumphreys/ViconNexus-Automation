from viconnexusapi import ViconNexus
import os
import numpy as np
import matplotlib.pyplot as plt
import heapq
from plate import Plate
import math


vicon = ViconNexus.ViconNexus()
player_file = "PLAY-06"

    # for file in os.path(f"C:/Users/ahumphreys/EXOS_Processing/{player_file}"):
    #     vicon.OpenTrial(file, 30)

file = fr"C:\Users\ahumphreys\EXOS_Processing\{player_file}\Cleat01\{player_file.replace('-', '-')}_Cleat01_Trial11"
vicon.OpenTrial(file, 30)

subject = vicon.GetSubjectNames()[0]

user_defined_region = vicon.GetTrialRegionOfInterest()

foot_contact_markers = []

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

markers = left_foot_markers + right_foot_markers
z_coords = {marker: [] for marker in markers}
y_coords = {marker: [] for marker in markers}
x_coords = {marker: [] for marker in markers}
z_velo = {marker: [] for marker in markers}
z_accel = {marker: [] for marker in markers}

for marker in markers:
    z_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[2] for frame in range(user_defined_region[0], user_defined_region[1]))
    y_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[1] for frame in range(user_defined_region[0], user_defined_region[1]))
    x_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[0] for frame in range(user_defined_region[0], user_defined_region[1]))
    z_velo[marker].extend(np.gradient(z_coords[marker])) 
    z_accel[marker].extend(np.diff(z_coords[marker], 2))


def calculate_bounding_box(i):
    min_x = min(x_coords[marker][i] for marker in right_foot_markers)
    max_x = max(x_coords[marker][i] for marker in right_foot_markers)
    min_y = min(y_coords[marker][i] for marker in right_foot_markers)
    max_y = max(y_coords[marker][i] for marker in right_foot_markers)
    right_box = (min_x, min_y, max_x, max_y)
    min_x = min(x_coords[marker][i] for marker in left_foot_markers)
    max_x = max(x_coords[marker][i] for marker in left_foot_markers)
    min_y = min(y_coords[marker][i] for marker in left_foot_markers)
    max_y = max(y_coords[marker][i] for marker in left_foot_markers)
    left_box = (min_x, min_y, max_x, max_y)
    return[right_box, left_box]


def find_cycles(marker: str = 'RD2P'):
    foot_down_frames = []
    def is_z_accel_peak(i, threshold: float = 4):
        return marker_z_accel[i] > threshold and marker_z_accel[i-1] < marker_z_accel[i] > marker_z_accel[i+1]
 
    def is_z_velo_trough(i, threshold: float = -4):
        return marker_z_velo[i] < threshold and marker_z_velo[i-1] > marker_z_velo[i] < marker_z_velo[i+1]
    
    z_accel_peak = z_velo_trough = False
    marker_z_accel = z_accel[marker]
    marker_z_velo = z_velo[marker]
    marker_z_pos = z_coords[marker]
 
    for i in range(1, len(marker_z_accel) - 1):
        if is_z_accel_peak(i):
            z_accel_peak = True    

        if is_z_velo_trough(i):
            z_velo_trough = True

        if z_accel_peak and z_velo_trough and marker_z_accel[i-1] > marker_z_velo[i-1] and marker_z_accel[i] < marker_z_velo[i] and marker_z_pos[i] < 120:
            foot_down_frames.append(i + user_defined_region[0])
            z_accel_peak = z_velo_trough = False

    return foot_down_frames

def plot_markers(markers):
    plt.figure(figsize=(10,6))

    for marker in markers:
        frames = np.arange(user_defined_region[0], user_defined_region[1])
        plt.plot(frames, z_coords[marker], label = marker + ' z coord')
        plt.plot(frames, z_velo[marker], label = marker + ' z velo')
        frames = np.arange(user_defined_region[0], user_defined_region[1] - 2)
        plt.plot(frames, z_accel[marker], label = marker + ' z accel')

    plt.xlabel('Frame')
    plt.title('Kinematics over time')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_forces(fp):
    plt.figure(figsize=(10,6))
    frames = np.arange(0, len(fp.fx)) 
    fp.calculate_gradient('x')
    fp.calculate_gradient('y')
    fp.calculate_gradient('z')
    plt.plot(frames, fp.fx, label = 'x force') 
    plt.plot(frames, fp.fy, label = 'y force') 
    plt.plot(frames, fp.fz, label = 'z force') 
    plt.plot(frames, fp.dx, label = 'dx') 
    plt.plot(frames, fp.dy, label = 'dy') 
    plt.plot(frames, fp.dz, label = 'dz') 

    plt.xlabel('Frame')
    plt.title('Force over time')
    plt.legend()
    plt.grid(True)
    plt.show()


def calculate_cycles(list1, list2, list3):
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
    print(points)
    for i in range(len(points)-1):
        diff = abs(points[i] - points[i+1])
        if diff <= 19 and not in_bounds:
            final_points.append(points[i])
            in_bounds = True
        elif diff > 19:
            in_bounds = False

    return final_points

def find_plate_data():
    strikes = []
    deviceIDs = vicon.GetDeviceIDs()
    for deviceID in deviceIDs:
            _, type, rate, _, force_plate, _ = vicon.GetDeviceDetails(deviceID)
            if type == 'ForcePlate':
                wt = force_plate.WorldT
                fx = vicon.GetDeviceChannel(deviceID, 1, 1)[0]
                fy = vicon.GetDeviceChannel(deviceID, 1, 2)[0]
                fz = vicon.GetDeviceChannel(deviceID, 1, 3)[0]
                fp = Plate(find_plate_name(wt), fx, fy, fz) 
                strikes.append(find_plate_strikes(fp))
    return strikes

def find_plate_strikes(fp):
    strike_intervals = []
    possible_strike = False
    fc = 0
    start_frame = end_frame = None
    for i in range(user_defined_region[0] * 10, user_defined_region[1] * 10):
        resultant =  math.sqrt(fp.fx[i] ** 2 + fp.fy[i] ** 2 + fp.fz[i] ** 2)
        if resultant > 20:
            if not possible_strike:
                possible_strike = True
                start_frame = i
            elif fc > 10:
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

def find_plate_matches(strike_intervals):
    plate_configs = {
        'Plate1' : [2712.0,300.0,0.0],
        'Plate2' : [2712.0,903.0,0.0], 
        'Plate3' : [2109.0,300.0,0.0], 
        'Plate4' : [2109.0,903.0,0.0],
        'Plate5' : [1506.0,300.0,0.0],
        'Plate6' : [1506.0,903.0,0.0],
        'Plate7' : [903.0,300.0,0.0],
        'Plate8' : [903.0,903.0,0.0],
        'Plate9': [300.0,300.0,0.0],
    }
    events = {'left': vicon.GetEvents(subject, 'Right', 'Foot Strike')[0], 'right': vicon.GetEvents(subject, 'Left', 'Foot Strike')[0]}
    for interval in strike_intervals:







def find_plate_name(wt):
    name = None
    if wt == [2712.0,300.0,0.0]:
        name = 'Plate1'
    elif wt == [2712.0,903.0,0.0]:
        name = 'Plate2'
    elif wt == [2109.0,300.0,0.0]:
       name = 'Plate3'
    elif wt == [2109.0,903.0,0.0]:
        name = 'Plate4'
    elif wt == [1506.0,300.0,0.0]:
        name = 'Plate5'  
    elif wt == [1506.0,903.0,0.0]:
        name = 'Plate6'
    elif wt == [903.0,300.0,0.0]:
        name = 'Plate7'
    elif wt == [903.0,903.0,0.0]:
        name = 'Plate8'
    elif wt == [300.0,300.0,0.0]:
        name = 'Plate9'
    if name:
        return name
    else:
        raise Exception("Not a valid force plate")

def main():
    print(calculate_cycles(find_cycles(right_foot_markers[0]), find_cycles(right_foot_markers[1]), find_cycles(right_foot_markers[2])))
    find_plate_data()

if __name__ == "__main__":
    main()


#vicon.SaveTrial()