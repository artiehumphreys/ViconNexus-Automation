from viconnexusapi import ViconNexus
import os
import numpy as np
import matplotlib.pyplot as plt

vicon = ViconNexus.ViconNexus()
player_file = "Play-30"
    # for file in os.path(f"C:/Users/ahumphreys/EXOS_Processing/{player_file}"):
    #     vicon.OpenTrial(file, 30)

file = fr"C:\Users\ahumphreys\EXOS_Processing\{player_file}\Cleat01\{player_file.replace('-', '')}_Cleat01_Trial07"
vicon.OpenTrial(file, 30)

subject = vicon.GetSubjectNames()[0]

user_defined_region = vicon.GetTrialRegionOfInterest()

foot_contact_markers = []

right_foot_markers = (
    'RD2P',
    'RD5P',
    # 'RHEE',
)

left_foot_markers = (
    'LD2P',
    'LD5P',
    # 'LHEE'
)

markers = left_foot_markers + right_foot_markers


z_coords = {marker: [] for marker in markers}
velo = {marker: [] for marker in markers}
accel = {marker: [] for marker in markers}
jerk = {marker: [] for marker in markers}
jounce = {marker: [] for marker in markers}

for marker in markers:
    z_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[2] for frame in range(user_defined_region[0], user_defined_region[1]))
for marker in markers:   
    velo[marker].extend(np.gradient(z_coords[marker]))
for marker in markers:   
    accel[marker].extend(np.diff(z_coords[marker], 2))
for marker in markers:   
    jerk[marker].extend(np.diff(z_coords[marker], 3))
for marker in markers:   
    jounce[marker].extend(np.diff(z_coords[marker], 4))

def find_cycles(marker: str = 'RD2P'):
    foot_down_frames = []
    foot_up_frames = []
    foot_down = False
    def is_accel_peak(i, threshold: float = 4):
        return marker_accel[i] > threshold and marker_accel[i-1] < marker_accel[i] > marker_accel[i+1]
 
    def is_velo_trough(i, threshold: float = -4):
        return marker_velo[i] < threshold and marker_velo[i-1] > marker_velo[i] < marker_velo[i+1]
    
    def is_velo_peak(i, threshold: float = 4):
        return marker_velo[i] > threshold 
        #and marker_velo[i-1] < marker_velo[i] > marker_velo[i+1]
    def is_jerk_trough(i, threshold = -0.75):
        return marker_jerk[i] < threshold and marker_jerk[i-1] > marker_jerk[i] < marker_jerk[i+1]
 
    accel_peak = velo_trough = velo_trough_plant = velo_peak = jerk_trough = False
    marker_accel = accel[marker]
    marker_velo = velo[marker]
    marker_jerk = jerk[marker]
 
    for i in range(1, len(marker_accel) - 1):
        if not foot_down:
            if is_accel_peak(i):
                accel_peak = True
        
            if is_velo_trough(i):
                velo_trough = True

            if accel_peak and velo_trough and marker_accel[i-1] > marker_velo[i-1] and marker_accel[i] < marker_velo[i]:
                foot_down_frames.append(i-1 + user_defined_region[0])
                #print("strike at " + str(i-1 + user_defined_region[0]))
                accel_peak = velo_trough = jerk_trough = False
                foot_down = True
        else:
            if is_velo_peak(i):
                velo_peak = True
        
            if is_velo_trough(i, -2.5):
                velo_trough_plant = True
            
            if is_jerk_trough(i):
                jerk_trough = True
            
            if velo_peak and jerk_trough:
                foot_up_frames.append(i+1 + user_defined_region[0])
                #print("foot up at " + str(i-1 + user_defined_region[0]))
                velo_peak = velo_trough_plant = jerk_trough = False
                foot_down = False

            elif velo_trough_plant and jerk_trough:
                foot_up_frames.append(i+1 + user_defined_region[0])
                #print("foot up at " + str(i-1 + user_defined_region[0]))
                velo_peak = velo_trough_plant = jerk_trough = False
                foot_down = False
    return foot_down_frames, foot_up_frames

    

def plot(marker: str = 'RD2P'):
    plt.figure(figsize=(10,6))


    frames = np.arange(user_defined_region[0], user_defined_region[1])
    plt.plot(frames, z_coords[marker])
    plt.plot(frames, velo[marker], label = 'velo')
    frames = np.arange(user_defined_region[0], user_defined_region[1] - 2)
    plt.plot(frames, accel[marker], label = 'accel')
    frames = np.arange(user_defined_region[0], user_defined_region[1] - 3)
    plt.plot(frames, jerk[marker], label = 'jerk')
    frames = np.arange(user_defined_region[0], user_defined_region[1] - 4)
    plt.plot(frames, jounce[marker], label = 'jounce')

    plt.xlabel('Frame')
    plt.title('Kinematics over time')
    plt.legend()
    plt.grid(True)
    plt.show()

def calculate_cycles(cycles_a, cycles_b):
    points = ([], [])
    for i in range(2):
        a_idx = b_idx = 0
        for _ in range(max(len(cycles_a[i]), len(cycles_b[i]))):
            diff = abs(cycles_a[i][a_idx] - cycles_b[i][b_idx])
            if diff <= 10:
                points[i].append((cycles_a[i][a_idx] + cycles_b[i][b_idx])//2)
                a_idx += 1
                b_idx += 1
            elif i == 0:
                if cycles_a[i][a_idx] < cycles_b[i][b_idx]:
                    points[i].append(cycles_a[i][a_idx])
                    a_idx += 1
                else:
                    points[i].append(cycles_b[i][b_idx])
                    b_idx += 1
            else:
                if cycles_a[i][a_idx] > cycles_b[i][b_idx]:
                    points[i].append(cycles_a[i][a_idx])
                    a_idx += 1
                else:
                    points[i].append(cycles_b[i][b_idx])
                    b_idx += 1

    return points


def main():
    print(find_cycles(right_foot_markers[0]), find_cycles(right_foot_markers[1]))
    print(calculate_cycles(find_cycles(right_foot_markers[0]), find_cycles(right_foot_markers[1])))
    plot('RD2P')

if __name__ == "__main__":
    main()


#vicon.SaveTrial()