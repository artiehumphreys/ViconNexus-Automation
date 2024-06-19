# pylint: disable=missing-module-docstring
import numpy as np
from scipy.spatial import ConvexHull
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
    """Class for a foot object, made up of the different markers of either the left or right foot"""

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
        points = np.array(
            [
                [self.x_coords[marker][i], self.y_coords[marker][i]]
                for marker in self.markers
            ]
        )
        box = ConvexHull(points)

        hull_points = points[box.vertices]

        min_x = min(hull_points[:, 0])
        max_x = max(hull_points[:, 0])
        min_y = min(hull_points[:, 1])
        max_y = max(hull_points[:, 1])
        return min_x, max_x, min_y, max_y

    def find_min_z(self, i):
        """Calculate the minimum z-coordinate given all of the markers"""
        return min(self.z_coords[marker][i] for marker in self.markers)

    def is_strike_in_plate(self, cop_x, cop_y, min_z, i):
        """Check if the foot is within 20cm of center of pressure and below a z threshold"""
        threshold = 200
        for marker in self.markers:
            if (
                self.x_coords[marker][i] - threshold
                <= cop_x
                <= self.x_coords[marker][i] + threshold
                and self.y_coords[marker][i] - threshold
                <= cop_y
                <= self.y_coords[marker][i] + threshold
                and self.z_coords[marker][i] < min_z + threshold / 8
            ):
                return True
        return False
