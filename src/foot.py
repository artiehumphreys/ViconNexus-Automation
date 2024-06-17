from vicon import Vicon

right_foot_markers = (
    "RD2P",
    "RD5P",
    "RHEE",
    "RLATH",
    "RD5M",
    "RD1M",
    "RP1M",
)

left_foot_markers = (
    "LD2P",
    "LD5P",
    "LHEE",
    "LLATH",
    "LD5M",
    "LD1M",
    "LP1M",
)


class Foot:
    def __init__(self, foot: str = "right"):
        self.markers = left_foot_markers if foot == "left" else right_foot_markers
        self.foot = foot
        self.z_coords = {marker: [] for marker in self.markers}
        self.y_coords = {marker: [] for marker in self.markers}
        self.x_coords = {marker: [] for marker in self.markers}
        self.vicon = Vicon()

        for marker in self.markers:
            self.z_coords[marker] = self.vicon.fetch_trajectory(marker)[2]
            self.y_coords[marker] = self.vicon.fetch_trajectory(marker)[1]
            self.x_coords[marker] = self.vicon.fetch_trajectory(marker)[0]

    def calculate_bounding_box(self, i):
        """Calculate a bounding box for the foot at a given frame using marker data"""
        min_x = min(self.x_coords[marker][i] for marker in self.markers)
        max_x = max(self.x_coords[marker][i] for marker in self.markers)
        min_y = min(self.y_coords[marker][i] for marker in self.markers)
        max_y = max(self.y_coords[marker][i] for marker in self.markers)
        box = (min_x, max_x, min_y, max_y)
        return box
            