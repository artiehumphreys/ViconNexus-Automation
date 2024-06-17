import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))
# pylint: disable=wrong-import-position
from plate import driver
from vicon import Vicon


# pylint: disable=too-many-locals
# pylint: disable=too-many-statements
class ForceMatrixCalculator:
    def __init__(self, results, plate_objs):
        print(results)
        self.results = results
        self.plate_objs = plate_objs
        self.vicon = Vicon()
        _, self.upper_bound = self.vicon.get_region_of_interest()
        a = 3 * np.pi / 2

        self.x270_matrix = [
            [1, 0, 0],
            [0, np.cos(a), np.sin(a)],
            [0, -np.sin(a), np.cos(a)],
        ]
        self.left_matrix = np.zeros((self.upper_bound * 10, 9), dtype="float")
        self.right_matrix = np.zeros((self.upper_bound * 10, 9), dtype="float")

    def find_force_matrix(self):
        """Helper method for finding force matrix at each frame"""
        for plate_obj in self.plate_objs:
            self.process_plate(plate_obj)
        return self.left_matrix, self.right_matrix

    def process_plate(self, plate_obj):
        """Filter plate data and prepare for information retreival"""
        plate_name = plate_obj.name
        #pylint: disable=attribute-defined-outside-init
        self.total_x_moment = np.zeros(self.upper_bound * 10)
        self.total_z_moment = np.zeros(self.upper_bound * 10)
        self.total_y_moment = np.zeros(self.upper_bound * 10)
        sides = ["left", "right"]

        if not any(self.results[plate_name][side] for side in sides):
            return

        for side in sides:
            intervals = self.results[plate_name][side]
            self.process_frames(plate_obj, side, intervals)

    def process_frames(self, plate_obj, side, intervals):
        """Calculate moment, cop, and force at each frame within user-defined region"""
        for interval in intervals:
            for j in range(interval[0] * 10, interval[1] * 10):
                rel_pos_on_plate, world_force, world_moment, world_raw_loc = (
                    self.compute_forces_and_moments(plate_obj, j)
                )
                self.total_x_moment[j] += rel_pos_on_plate[1] * world_force[2]
                self.total_z_moment[j] = -(
                    rel_pos_on_plate[1] * world_force[0]
                    - rel_pos_on_plate[0] * world_force[1]
                )
                self.total_y_moment[j] += -rel_pos_on_plate[0] * world_force[2]

                cop_x, cop_y = self.calculate_overall_center_of_pressure(
                    plate_obj, interval, j
                )

                # Only for OpenSim:
                adj_moment = np.zeros(3)
                adj_moment[2] = (
                    world_moment[2]
                    + world_force[0] * world_raw_loc[1]
                    - world_force[1] * world_raw_loc[0]
                ) / 1000
                force_cols = -world_force @ self.x270_matrix  # type: ignore
                torque_cols = -adj_moment @ self.x270_matrix
                cop_cols = np.array([cop_x / 1000, cop_y / 1000, 0]) @ self.x270_matrix

                self.update_matrices(side, j, force_cols, torque_cols, cop_cols)

    def update_matrices(self, side, j, force_cols, torque_cols, cop_cols):
        """Update matrices for each foot based on calculations"""
        if side == "left":
            self.left_matrix[j, :3] += force_cols
            self.left_matrix[j, 3:6] += cop_cols
            self.left_matrix[j, 6:] += torque_cols
        else:
            self.right_matrix[j, :3] += force_cols
            self.right_matrix[j, 3:6] += cop_cols
            self.right_matrix[j, 6:] += torque_cols


    def compute_forces_and_moments(self, plate_obj, j):
        """Get plate data and perform initial matrix rotation to properly format data"""
        fx = plate_obj.fx[j - 10]
        fy = plate_obj.fy[j - 10]
        fz = plate_obj.fz[j - 10]
        copx = plate_obj.copx[j - 10]
        copy = plate_obj.copy[j - 10]
        copz = plate_obj.copz[j - 10]
        copx_raw = plate_obj.copx_raw[j - 10]
        copy_raw = plate_obj.copy_raw[j - 10]
        copz_raw = plate_obj.copz_raw[j - 10]

        mx = plate_obj.mx[j - 10]
        my = plate_obj.my[j - 10]
        mz = plate_obj.mz[j - 10]
        wr = plate_obj.wr

        force_on_plate = np.array([fx, fy, fz])
        moment_on_plate = np.array([mx, my, mz])
        rel_pos_on_plate = np.array([copx, copy, copz])
        raw_pos_on_plate = np.array([copx_raw, copy_raw, copz_raw])

        rotw = np.array([wr[:3], wr[3:6], wr[6:]])
        world_force = rotw @ force_on_plate
        world_moment = rotw @ moment_on_plate
        world_raw_loc = rotw @ raw_pos_on_plate

        return rel_pos_on_plate, world_force, world_moment, world_raw_loc

    def calculate_overall_center_of_pressure(self, plate_obj, interval, frame):
        """Calculate center of pressure at a given frame for a plate"""
        sum_fz = 0
        sum_fy = 0
        sum_fz_copx = 0
        sum_fz_copy = 0
        sum_fy_copz = 0
        if interval[0] * 10 - 10 <= frame < interval[1] * 10:
            fy = plate_obj.fy[frame - 10]
            fz = plate_obj.fz[frame - 10]
            copx = plate_obj.copx[frame - 10]
            copy = plate_obj.copy[frame - 10]
            copz = plate_obj.copz[frame - 10]

            sum_fz += fz
            sum_fy += fy
            sum_fz_copx += copx * fz
            sum_fz_copy += copy * fz
            sum_fy_copz += copz * fy

        if sum_fz != 0:
            cop_overall_x = sum_fz_copx / sum_fz
            cop_overall_y = sum_fz_copy / sum_fz
        else:
            cop_overall_x = np.nan
            cop_overall_y = np.nan

        return cop_overall_x, cop_overall_y

# pylint: disable=missing-function-docstring
def main():
    np.set_printoptions(threshold=sys.maxsize)
    results, plate_objs = driver()
    calc = ForceMatrixCalculator(results, plate_objs)
    left, _ = calc.find_force_matrix()
    np.savetxt("res.csv", left, delimiter=",")  # type: ignore


if __name__ == "__main__":
    main()
