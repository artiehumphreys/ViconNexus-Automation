import numpy as np
import matplotlib.pyplot as plt
from plate import Plate
import sys

vicon = vicon()

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

def find_force_matrix(results):
    global fp_data
    a = 3 * np.pi / 2
    temp_side = None

    x270_matrix = [[1, 0, 0],  [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]]
    summ = 0
    left_matrix = np.zeros((user_defined_region[1] * 10, 9), dtype='float')
    right_matrix = np.zeros((user_defined_region[1] * 10, 9), dtype='float')
    total_y_force = 0
    processed_frames_right = set()
    processed_frames_left = set()
    for plate in results:
        total_x_moment = np.zeros(user_defined_region[1] * 10)
        total_z_moment = np.zeros(user_defined_region[1] * 10)
        total_y_moment = np.zeros(user_defined_region[1] * 10)
        if not results[plate]['left'] and not results[plate]['right']:
            continue
        for side in ['left', 'right']:
            intervals = results[plate][side]
            for interval in intervals:
                for j in range(interval[0] * 10, interval[1] * 10):
                    # 1 frame offset (10 plate frames)
                    fx = fp_data[plate]['fx'][j- 10]
                    fy = fp_data[plate]['fy'][j- 10]
                    fz = fp_data[plate]['fz'][j- 10]
                    copx = fp_data[plate]['copx'][j- 10]
                    copy = fp_data[plate]['copy'][j- 10]
                    copz = fp_data[plate]['copz'][j- 10]
                    mx = fp_data[plate]['mx'][j- 10]
                    my = fp_data[plate]['my'][j- 10]
                    mz = fp_data[plate]['mz'][j- 10]
                    wr = fp_data[plate]['wr']
 
                    force_on_plate = np.array([fx, fy, fz])
                    moment_on_plate = np.array([mx, my, mz])
                    rel_pos_on_plate = np.array([copx, copy, copz])

                    rotW = [wr[:3], wr[3:6], wr[6:]]
 
                    world_force = rotW @ force_on_plate
                    world_moment = rotW @ moment_on_plate
                    world_loc = rotW @ rel_pos_on_plate
 
                    # Only for OpenSim:
                    # adj_moment = np.zeros(3)
                    # adj_moment[2] = world_moment[2] + world_force[0] * world_loc[1] - world_force[1] * world_loc[0]
                    # force_cols = -world_force 
                    # torque_cols = -adj_moment @ x270_matrix
                    # cop_cols = world_loc @ x270_matrix

                    total_y_force += world_force[1]
                    total_x_moment[j] += rel_pos_on_plate[1] * world_force[2]
                    total_z_moment[j] = -(rel_pos_on_plate[1] * world_force[0] - rel_pos_on_plate[0] * world_force[1])
                    total_y_moment[j] += -rel_pos_on_plate[0] * world_force[2]

                    CoP_x, CoP_y, CoP_z = calculate_overall_center_of_pressure(results, plate, side, interval, j)

                    if side == 'left':
                        left_matrix[j, :3] += world_force
                        left_matrix[j, 3:6] += [total_x_moment[j], total_y_moment[j], total_z_moment[j]]
                        left_matrix[j, 6:] = [CoP_x, CoP_y, CoP_z]
                        processed_frames_left.add(j)
                    else:
                        right_matrix[j, :3] += world_force
                        right_matrix[j, 3:6] += [total_x_moment[j], total_y_moment[j], total_z_moment[j]]
                        right_matrix[j, 6:] = [CoP_x, CoP_y, CoP_z]
                        if j == 7820:
                            print(world_moment)
                            print(total_x_moment[j])
                        processed_frames_right.add(j)
 
    return left_matrix, right_matrix


def calculate_overall_center_of_pressure(results, plate, side, interval, frame):
    global fp_data
    sum_fz = 0
    sum_fy = 0
    sum_fx_copx = 0
    sum_fy_copy = 0
    sum_fz_copx = 0
    sum_fz_copy = 0
    sum_fy_copz = 0
    sum_fz_copz = 0 
    if interval[0] * 10 - 10 <= frame < interval[1] * 10:
        fx = fp_data[plate]['fx'][frame - 10]
        fy = fp_data[plate]['fy'][frame - 10]
        fz = fp_data[plate]['fz'][frame - 10]
        copx = fp_data[plate]['copx'][frame - 10]
        copy = fp_data[plate]['copy'][frame - 10]
        copz = fp_data[plate]['copz'][frame - 10]
    
        sum_fz += fz
        sum_fy += fy
        sum_fz_copx += copx * fz
        sum_fz_copy += copy * fz
        sum_fy_copz += copz * fy
 
    if sum_fz != 0:
        CoP_overall_x = sum_fz_copx / sum_fz
        CoP_overall_y = sum_fz_copy / sum_fz
    else:
        CoP_overall_x = np.nan
        CoP_overall_y = np.nan
 
    if sum_fy != 0:
        CoP_overall_z = sum_fy_copz / sum_fy
    else:
        CoP_overall_z = np.nan
 
    return CoP_overall_x, CoP_overall_y, CoP_overall_z

def main():
    np.set_printoptions(threshold=sys.maxsize)
    results = find_plate_matches(find_plate_data())
    left, right = find_force_matrix(results)
    # np.savetxt('right_foot_results.csv',  right, delimiter=",", header="Fx, Fy, Fz, Mx, My, Mz, CoPx, CoPy, CoPz")
    print(right[6200])

if __name__ == "__main__":
    main()


#vicon.SaveTrial()