from viconnexusapi import ViconNexus

class Vicon():
    def __init__(self):
        self.vicon = ViconNexus.ViconNexus()
        player_file = "Play-07"

        file = fr"C:\Users\ahumphreys\EXOS_Processing\{player_file}\Cleat02\{player_file.replace('-', '')}_Cleat02_Trial05"
        self.vicon.OpenTrial(file, 30)

        self.subject = self.vicon.GetSubjectNames()[0]

        self.user_defined_region = self.vicon.GetTrialRegionOfInterest()
    
    def get_region_of_interest(self):
        return self.user_defined_region
    
    def fetch_trajectory(self, marker) -> [list, list, list]:
        positions = []
        for frame in range(self.user_defined_region[0], self.user_defined_region[1]):
            positions.append(self.vicon.GetTrajectoryAtFrame(self.subject, marker, frame))
        return positions