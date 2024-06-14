from viconnexusapi import ViconNexus

class Vicon():
    def __init__(self):
        self.vicon = ViconNexus.ViconNexus()
        player_file = "Play-07"

        file = fr"C:\Users\ahumphreys\EXOS_Processing\{player_file}\Cleat02\{player_file.replace('-', '')}_Cleat02_Trial05"
        self.vicon.OpenTrial(file, 30)

        self.subject = self.vicon.GetSubjectNames()[0]

        self.user_defined_region = self.vicon.GetTrialRegionOfInterest()
        self.strike_events = None
        self.off_events = None
        self.fetch_events()
    
    def get_region_of_interest(self):
        return self.user_defined_region
    
    def fetch_trajectory(self, marker):
        positions = self.vicon.GetTrajectory(self.subject, marker)
        return positions
    
    def fetch_events(self):
        self.strike_events = {'right': self.vicon.GetEvents(self.subject, 'Right', 'Foot Strike')[0], 'left': self.vicon.GetEvents(self.subject, 'Left', 'Foot Strike')[0]}
        self.off_events = {'right': self.vicon.GetEvents(self.subject, 'Right', 'Foot Off')[0], 'left': self.vicon.GetEvents(self.subject, 'Left', 'Foot Off')[0]}