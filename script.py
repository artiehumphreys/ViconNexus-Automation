from viconnexusapi import ViconNexus
import os
import numpy as np
import matplotlib.pyplot as plt

vicon = ViconNexus.ViconNexus()
player_file = "Play-30"
    # for file in os.path(f"C:/Users/ahumphreys/EXOS_Processing/{player_file}"):
    #     vicon.OpenTrial(file, 30)

file = fr"C:\Users\ahumphreys\EXOS_Processing\{player_file}\Cleat01\{player_file.replace('-', '')}_Cleat01_Trial06"
vicon.OpenTrial(file, 30)

subject = vicon.GetSubjectNames()[0]

user_defined_region = vicon.GetTrialRegionOfInterest()

foot_contact_markers = []

right_foot_markers = (
    'RD2P',
    'RD5P',
    'RHEE',
    'RINFH'
)

left_foot_markers = (
    'LD2P',
    'LD5P',
    'LHEE'
)


z_coords = {marker: [] for marker in right_foot_markers}
velo = {marker: [] for marker in right_foot_markers}
accel = {marker: [] for marker in right_foot_markers}
jerk = {marker: [] for marker in right_foot_markers}
jounce = {marker: [] for marker in right_foot_markers}

for marker in right_foot_markers:
    z_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[2] for frame in range(user_defined_region[0], user_defined_region[1]))
for marker in right_foot_markers:   
    velo[marker].extend(np.gradient(z_coords[marker]))
for marker in right_foot_markers:   
    accel[marker].extend(np.diff(z_coords[marker], 2))
for marker in right_foot_markers:   
    jerk[marker].extend(np.diff(z_coords[marker], 3))
for marker in right_foot_markers:   
    jounce[marker].extend(np.diff(z_coords[marker], 4))

is_striking = True
def find_strikes(marker: str = 'RHEE'):
    def is_accel_peak(i, threshold: float =5):
        return marker_accel[i] > threshold and marker_accel[i-1] < marker_accel[i] > marker_accel[i+1]
 
    def is_velo_trough(i, threshold: float =-5):
        return marker_velo[i] < threshold and marker_velo[i-1] > marker_velo[i] < marker_velo[i+1]
 
    def is_velo_peak(i):
        return marker_velo[i] > 5 and marker_velo[i-1] < marker_velo[i] > marker_velo[i+1]
 
    foot_down = False
    accel_peak = velo_trough_for_plant = velo_trough_for_footup = velo_peak = accel_peak_for_footup = False
 
    marker_accel = accel[marker]
    marker_velo = velo[marker]
    marker_jerk = jerk[marker]

    print(is_velo_trough(679 - user_defined_region[0], -2.5), is_accel_peak(680 - user_defined_region[0], 2.5))
 
    for i in range(1, len(marker_accel) - 1):
        if is_accel_peak(i):
            accel_peak = True
        if foot_down and is_accel_peak(i, 0.75):
            accel_peak_for_footup = True
 
        if is_velo_trough(i):
            velo_trough_for_plant = True
        if foot_down and is_velo_trough(i, -0.75):
            velo_trough_for_footup = True

        if not foot_down and accel_peak and velo_trough_for_plant and marker_accel[i-1] > marker_velo[i-1] and marker_accel[i] < marker_velo[i]:
            print("strike at " + str(i-1 + user_defined_region[0]))
            accel_peak = velo_trough_for_plant = False
            foot_down = True
        elif foot_down and accel_peak_for_footup and velo_trough_for_footup and marker_accel[i-1] > marker_velo[i-1] and marker_accel[i] < marker_velo[i]:
            print("foot up action at " + str(i + user_defined_region[0]))
            foot_down = False
            accel_peak_for_footup = velo_trough_for_footup = False

        if foot_down and is_velo_peak(i):
            velo_peak = True
    
        if foot_down and velo_peak and marker_jerk[i] > marker_accel[i]:
            print("foot up at " + str(i + user_defined_region[0]))
            foot_down = False
            velo_peak = False

        
        # if not accel_peak and not velo_trough_for_plant and not velo_peak and marker_velo[i] > 5:
        #     velo_peak = False
        # if velo_peak and marker_velo[i-1] > marker_jerk[i-1] and marker_velo[i] < marker_jerk[i]:
        #     print("foot up at " + str(i - 1 + user_defined_region[0]))
        #     velo_peak = False

def plot():
    plt.figure(figsize=(10,6))


    frames = np.arange(user_defined_region[0], user_defined_region[1])
    plt.plot(frames, z_coords['RD2P'])
    plt.plot(frames, velo['RD2P'], label = 'velo')
    frames = np.arange(user_defined_region[0], user_defined_region[1] - 2)
    plt.plot(frames, accel['RD2P'], label = 'accel')
    frames = np.arange(user_defined_region[0], user_defined_region[1] - 3)
    plt.plot(frames, jerk['RD2P'], label = 'jerk')

    plt.xlabel('Frame')
    plt.title('Kinematics over time')
    plt.legend()
    plt.grid(True)
    plt.show()

find_strikes('RD2P')
plot()