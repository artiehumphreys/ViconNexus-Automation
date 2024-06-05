from viconnexusapi import ViconNexus
import os

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

for marker in right_foot_markers:
    z_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[2] for frame in range(user_defined_region[0], user_defined_region[1]))
print(z_coords)
