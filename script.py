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

for marker in right_foot_markers:
    z_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[2] for frame in range(user_defined_region[0], user_defined_region[1]))
for marker in right_foot_markers:   
    velo[marker].extend(np.gradient(z_coords[marker]))
for marker in right_foot_markers:   
    accel[marker].extend(np.diff(z_coords[marker], 2))

is_striking = True
def find_strikes(marker: str = 'RHEE'):
    has_peak = has_trough = False
    marker_accel = accel[marker]
    marker_velo = velo[marker]
    for i in range(len(marker_accel)):
        if marker_accel[i] > 5 and marker_accel[i-1] < marker_accel[i] > marker_accel[i+1]:
            has_peak = True
        if marker_velo[i] < -5 and marker_velo[i-1] > marker_velo[i] < marker_velo[i+1]:
            has_trough = True
        if has_peak and has_trough and marker_accel[i-1] > marker_velo[i-1] and marker_accel[i] < marker_velo[i]:
            print("strike at " + str(i-1 + user_defined_region[0]))
            has_peak = False
            has_trough = False

def plot():
    plt.figure(figsize=(10,6))


    frames = np.arange(user_defined_region[0], user_defined_region[1])
    plt.plot(frames, z_coords['RD2P'])
    plt.plot(frames, velo['RD2P'], label = 'velo')
    frames = np.arange(user_defined_region[0], user_defined_region[1] - 2)
    plt.plot(frames, accel['RD2P'], label = 'accel')

    plt.xlabel('Frame')
    plt.title('Kinematics over time')
    plt.legend()
    plt.grid(True)
    plt.show()

find_strikes('RD2P')
plot()