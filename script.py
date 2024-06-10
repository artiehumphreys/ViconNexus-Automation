from viconnexusapi import ViconNexus
import os
import numpy as np
import matplotlib.pyplot as plt
import heapq
from plate import Plate
import math
import sys


vicon = ViconNexus.ViconNexus()
player_file = "Play-07"

    # for file in os.path(f"C:/Users/ahumphreys/EXOS_Processing/{player_file}"):
    #     vicon.OpenTrial(file, 30)

file = fr"C:\Users\ahumphreys\EXOS_Processing\{player_file}\Cleat02\{player_file.replace('-', '')}_Cleat02_Trial05"
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
    right_box = (min_x, max_x, min_y, max_y)
    min_x = min(x_coords[marker][i] for marker in left_foot_markers)
    max_x = max(x_coords[marker][i] for marker in left_foot_markers)
    min_y = min(y_coords[marker][i] for marker in left_foot_markers)
    max_y = max(y_coords[marker][i] for marker in left_foot_markers)
    left_box = (min_x, max_x, min_y, max_y)
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
    for i in range(len(points)-1):
        diff = abs(points[i] - points[i+1])
        if diff <= 19 and not in_bounds:
            final_points.append(points[i])
            in_bounds = True
        elif diff > 19:
            in_bounds = False

    return final_points

fp_data = {}
def find_plate_data():
    global fp
    strikes = []
    deviceIDs = vicon.GetDeviceIDs()
    for deviceID in deviceIDs:
            _, type, rate, _, force_plate, _ = vicon.GetDeviceDetails(deviceID)
            if type == 'ForcePlate':
                wt = force_plate.WorldT
                fx = vicon.GetDeviceChannel(deviceID, 1, 1)[0]
                fy = vicon.GetDeviceChannel(deviceID, 1, 2)[0]
                fz = vicon.GetDeviceChannel(deviceID, 1, 3)[0]
                copx = vicon.GetDeviceChannel(deviceID, 2, 1)[0]
                copy = vicon.GetDeviceChannel(deviceID, 2, 2)[0]
                copz = vicon.GetDeviceChannel(deviceID, 2, 3)[0]
                mx = vicon.GetDeviceChannel(deviceID, 3, 1)[0]
                my = vicon.GetDeviceChannel(deviceID, 3, 2)[0]
                mz = vicon.GetDeviceChannel(deviceID, 3, 3)[0]
                name = find_plate_name(wt)
                if name:
                    fp_data[name] = {
                        'fx': fx,
                        'fy': fy,
                        'fz': fz,
                        'copx': copx,
                        'copy': copy,
                        'copz': copz,
                        'mx': mx,
                        'my': my,
                        'mz': mz
                    }
                fp = Plate(name, fx, fy, fz)
                strike = find_plate_strikes(fp)
                if strike:
                    strikes.append((strike, name))
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

def find_plate_matches(strike_intervals):
    results = {plate:{'left':[], 'right':[]} for plate in plate_configs.keys()}
    plate_bounds = [(coords[0] - 300, coords[0] + 300, coords[1] - 300, coords[1] + 300) for coords in plate_configs.values()]
    strike_events = {'right': vicon.GetEvents(subject, 'Right', 'Foot Strike')[0], 'left': vicon.GetEvents(subject, 'Left', 'Foot Strike')[0]}
    off_events = {'right': vicon.GetEvents(subject, 'Right', 'Foot Off')[0], 'left': vicon.GetEvents(subject, 'Left', 'Foot Off')[0]}

    for foot in strike_events:
        for i in range(len(off_events[foot])):
            for j in range(strike_events[foot][i], off_events[foot][i]):
                bbox = calculate_bounding_box(j - user_defined_region[0])[0] if foot == 'right' else calculate_bounding_box(j - user_defined_region[0])[1]
                min_x, max_x, min_y, max_y = bbox
                # foot not in bounds of the plates
                if (2712 < min_x and 300 > max_x and 903 < min_y and 0 > max_y):
                    continue
                for plate in plate_bounds:
                    plate_name = find_plate_key(plate[0], plate[2])
                    if plate_name and is_intersecting(bbox, plate) and frame_in_strike_interval(j, strike_intervals, plate_name):
                        results[plate_name][foot].append(j)
    results = format_results(results)
    return results

def format_results(results):
    for plate in results:
        left_intervals = []
        right_intervals = []
        left = results[plate]['left']
        right = results[plate]['right']
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
            results[plate]['left'] = left_intervals
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
            results[plate]['right'] = right_intervals
    return results

def frame_in_strike_interval(j, strike_intervals, plate):
    for intervals, each_plate in strike_intervals:
        for interval in intervals:
            start = interval[0] // 10 
            end = interval[1] // 10 
            if (each_plate == plate and start <= j <= end):
                return True
    return False

def is_intersecting(box1, box2):
    min_x1, max_x1, min_y1, max_y1 = box1
    min_x2, max_x2, min_y2, max_y2 = box2
    x_overlap = not(max_x1 < min_x2 or max_x2 < min_x1)
    y_overlap = not(max_y1 < min_y2 or max_y2 < min_y1)
    return x_overlap and y_overlap

def find_plate_key(min_x, min_y):
    for key, value in plate_configs.items():
        if value[0] - 300 <= min_x <= value[0] + 300 and value[1] - 300 <= min_y <= value[1] + 300:
            return key
    return None

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

def find_force_matrix(results):
    global fp_data
    rotation_matrix = np.eye(3)
    translation_vector = np.array([0,0,0])
    x270_matrix = np.array([[0,-1,0], [1,0,0], [0,0,-1]])

    left_matrix = np.zeros((user_defined_region[1] * 10, 9), dtype='float')
    right_matrix = np.zeros((user_defined_region[1] * 10, 9), dtype='float')
    for plate in results:
        if not results[plate]['left'] and not results[plate]['right']:
            continue
        for side in ['left', 'right']:
            intervals = results[plate][side]
            for interval in intervals:
                for j in range(interval[0] * 10, interval[1] * 10):
                    fx = fp_data[plate]['fx'][j- 10]
                    fy = fp_data[plate]['fy'][j- 10]
                    fz = fp_data[plate]['fz'][j- 10]
                    copx = fp_data[plate]['copx'][j- 10]
                    copy = fp_data[plate]['copy'][j- 10]
                    copz = fp_data[plate]['copz'][j- 10]
                    mx = fp_data[plate]['mx'][j- 10]
                    my = fp_data[plate]['my'][j- 10]
                    mz = fp_data[plate]['mz'][j- 10]
 
                    force_on_plate = np.array([fx, fy, fz])
                    moment_on_plate = np.array([mx, my, mz])
                    rel_pos_on_plate = np.array([copx, copy, copz])
 
                    world_force = np.dot(force_on_plate, rotation_matrix.T)
                    world_moment = np.dot(moment_on_plate, rotation_matrix.T)
                    world_loc = np.dot(rel_pos_on_plate, rotation_matrix.T) + translation_vector
 
                    adj_moment = np.zeros(3)
                    adj_moment[2] = world_moment[2] + world_force[0] * world_loc[1] - world_force[1] * world_loc[0]
 
                    force_cols = np.dot(-world_force, x270_matrix)
                    torque_cols = np.dot(-adj_moment / 1000, x270_matrix)
                    cop_cols = np.dot(world_loc / 1000, x270_matrix)
                    if side == 'left':
                        left_matrix[j, :3] += force_cols
                        left_matrix[j, 3:6] += cop_cols
                        left_matrix[j, 6:] += torque_cols
                    else:
                        right_matrix[j, :3] += force_cols
                        right_matrix[j, 3:6] += cop_cols
                        right_matrix[j, 6:] += torque_cols
 
    return left_matrix, right_matrix
def main():
    np.set_printoptions(threshold=sys.maxsize)

    # print(calculate_cycles(find_cycles(right_foot_markers[0]), find_cycles(right_foot_markers[1]), find_cycles(right_foot_markers[2])))
    results = find_plate_matches(find_plate_data())
    left, right = find_force_matrix(results)
    print(right[6160], right[6170], right[6180], right[6190], right[6200])

if __name__ == "__main__":
    main()


#vicon.SaveTrial()