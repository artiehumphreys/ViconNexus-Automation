from vicon import Vicon

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

class Foot:
    def __init__(self, foot: str = "right"):
        self.markers = left_foot_markers if foot == 'left' else right_foot_markers
        self.foot = foot
        self.z_coords = {marker: [] for marker in self.markers}
        self.y_coords = {marker: [] for marker in self.markers}
        self.x_coords = {marker: [] for marker in self.markers}
        self.vicon = Vicon()

        lower, upper = self.vicon.get_region_of_interest()

        for marker in self.markers:
            self.z_coords[marker].extend(self.vicon.vicon.GetTrajectoryAtFrame(self.vicon.subject, marker, frame)[2] for frame in range(lower, upper))
            self.y_coords[marker].extend(self.vicon.vicon.GetTrajectoryAtFrame(self.vicon.subject, marker, frame)[1] for frame in range(lower, upper))
            self.x_coords[marker].extend(self.vicon.vicon.GetTrajectoryAtFrame(self.vicon.subject, marker, frame)[0] for frame in range(lower, upper))

    def calculate_bounding_box(self, i):
        if self.foot == 'left':
            min_x = min(self.x_coords[marker][i] for marker in left_foot_markers)
            max_x = max(self.x_coords[marker][i] for marker in left_foot_markers)
            min_y = min(self.y_coords[marker][i] for marker in left_foot_markers)
            max_y = max(self.y_coords[marker][i] for marker in left_foot_markers)
            box = (min_x, max_x, min_y, max_y)
        else:
            min_x = min(self.x_coords[marker][i] for marker in right_foot_markers)
            max_x = max(self.x_coords[marker][i] for marker in right_foot_markers)
            min_y = min(self.y_coords[marker][i] for marker in right_foot_markers)
            max_y = max(self.y_coords[marker][i] for marker in right_foot_markers)
            box = (min_x, max_x, min_y, max_y)
        return box