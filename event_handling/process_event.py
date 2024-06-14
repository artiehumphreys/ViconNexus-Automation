import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from vicon import Vicon
from plate import driver, Plate
import numpy as np

def find_force_matrix(results):
    print(results)
    vicon = Vicon()
    _, upper_bound = vicon.get_region_of_interest()
    a = 3 * np.pi / 2

    x270_matrix = [[1, 0, 0],  [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]]
    left_matrix = np.zeros((upper_bound * 10, 9), dtype='float')
    right_matrix = np.zeros((upper_bound * 10, 9), dtype='float')
    total_y_force = 0
    processed_frames_right = set()
    processed_frames_left = set()
    for plate in results:
        fp = Plate(plate)
        total_x_moment = np.zeros(upper_bound * 10)
        total_z_moment = np.zeros(upper_bound * 10)
        total_y_moment = np.zeros(upper_bound * 10)
        if not results[plate]['left'] and not results[plate]['right']:
            continue
        for side in ['left', 'right']:
            intervals = results[plate][side]
            for interval in intervals:
                for j in range(interval[0] * 10, interval[1] * 10):
                    # 1 frame offset (10 plate frames)
                    fx = fp.fx[j- 10]
                    fy = fp.fy[j- 10]
                    fz = fp.fz[j- 10]
                    copx = fp.copx[j- 10]
                    copy = fp.copy[j- 10]
                    copz = fp.copz[j- 10]
                    mx = fp.mx[j- 10]
                    my = fp.my[j- 10]
                    mz = fp.mz[j- 10]
                    wr = fp.wr
 
                    force_on_plate = np.array([fx, fy, fz])
                    moment_on_plate = np.array([mx, my, mz])
                    rel_pos_on_plate = np.array([copx, copy, copz])

                    rotW = [wr[:3], wr[3:6], wr[6:]]
 
                    world_force = rotW @ force_on_plate
                    world_moment = rotW @ moment_on_plate
                    world_loc = rotW @ rel_pos_on_plate
 
                    # Only for OpenSim:
                    adj_moment = np.zeros(3)
                    adj_moment[2] = world_moment[2] + world_force[0] * world_loc[1] - world_force[1] * world_loc[0]
                    force_cols = -world_force 
                    torque_cols = -adj_moment @ x270_matrix
                    cop_cols = world_loc @ x270_matrix

                    total_y_force += world_force[1]
                    total_x_moment[j] += rel_pos_on_plate[1] * world_force[2]
                    total_z_moment[j] = -(rel_pos_on_plate[1] * world_force[0] - rel_pos_on_plate[0] * world_force[1])
                    total_y_moment[j] += -rel_pos_on_plate[0] * world_force[2]

                    CoP_x, CoP_y, CoP_z = calculate_overall_center_of_pressure(fp, interval, j)

                    if side == 'left':
                        left_matrix[j, :3] += world_force
                        if j in processed_frames_left:
                            left_matrix[j, 3:6] += world_moment
                        else:
                            left_matrix[j, 3:6] += [total_x_moment[j], total_y_moment[j], total_z_moment[j]]
                        left_matrix[j, 6:] = [CoP_x, CoP_y, CoP_z]
                        processed_frames_left.add(j)
                    else:
                        right_matrix[j, :3] += world_force
                        if j in processed_frames_right:
                            right_matrix[j, 3:6] += world_moment
                        else:
                            right_matrix[j, 3:6] += [total_x_moment[j], total_y_moment[j], total_z_moment[j]]
                        right_matrix[j, 6:] = [CoP_x, CoP_y, CoP_z]
                        processed_frames_right.add(j)
    return left_matrix, right_matrix


def calculate_overall_center_of_pressure(fp, interval, frame):
    sum_fz = 0
    sum_fy = 0
    sum_fz_copx = 0
    sum_fz_copy = 0
    sum_fy_copz = 0
    if interval[0] * 10 - 10 <= frame < interval[1] * 10:
        fy = fp.fy[frame - 10]
        fz = fp.fz[frame - 10]
        copx = fp.copx[frame - 10]
        copy = fp.copy[frame - 10]
        copz = fp.copz[frame - 10]
    
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
    results = driver()
    fm = find_force_matrix(results)
    f = open('res.csv', 'w')
    f.write(str(fm[0]))

if __name__ == "__main__":
    main()