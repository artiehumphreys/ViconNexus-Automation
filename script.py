from viconnexusapi import ViconNexus
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import argrelextrema

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

def find_cycles(marker: str = 'RD2P'):
    foot_down = False
    def is_accel_peak(i, threshold: float =5):
        return marker_accel[i] > threshold and marker_accel[i-1] < marker_accel[i] > marker_accel[i+1]
 
    def is_velo_trough(i, threshold: float =-5):
        return marker_velo[i] < threshold and marker_velo[i-1] > marker_velo[i] < marker_velo[i+1]
    
    def is_velo_peak(i, threshold: float =5):
        return marker_velo[i] > threshold and marker_velo[i-1] < marker_velo[i] > marker_velo[i+1]
 
    accel_peak = velo_trough = velo_trough_plant = velo_peak = False
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
                print("strike at " + str(i-1 + user_defined_region[0]))
                accel_peak = velo_trough = False
                foot_down = True
        else:
            if is_velo_peak(i):
                velo_peak = True
        
            if is_velo_trough(i, -2.5):
                velo_trough_plant = True
            
            if velo_peak and marker_jerk[i] > 0:
                print("foot up at " + str(i-1 + user_defined_region[0]))
                velo_peak = velo_trough_plant = False
                foot_down = False

            elif velo_trough_plant and marker_jerk[i] > 0:
                print("foot up at " + str(i-1 + user_defined_region[0]))
                velo_peak = velo_trough_plant = False
                foot_down = False

    

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

find_cycles()
plot('RD2P')