import math
import copy
import numpy as np
import matplotlib.pyplot as plt
from vicon import Vicon
from foot import Foot

plate_configs = {
    (2712.0, 300.0, 0.0): "Plate1",
    (2712.0, 903.0, 0.0): "Plate2",
    (2109.0, 300.0, 0.0): "Plate3",
    (2109.0, 903.0, 0.0): "Plate4",
    (1506.0, 300.0, 0.0): "Plate5",
    (1506.0, 903.0, 0.0): "Plate6",
    (903.0, 300.0, 0.0): "Plate7",
    (903.0, 903.0, 0.0): "Plate8",
    (300.0, 300.0, 0.0): "Plate9",
}

left_foot = Foot("left")
right_foot = Foot("right")


class Plate:
    def __init__(self, name, vicon):
        self.name = name
        self.vicon = vicon
        self.wt = ()
        self.fx = []
        self.fy = []
        self.fz = []
        self.dx = []
        self.dy = []
        self.dz = []
        self.copx = []
        self.copy = []
        self.copz = []
        self.mx = []
        self.my = []
        self.mz = []
        self.fetch_plate_data()

    def __str__(self):
        return self.name

    def calculate_gradient(self, dimension="x"):
        """(For gait detection) Calculate gradient of given axis"""
        if self.fx and self.fy and self.fz:
            match dimension:
                case "x":
                    self.dx = np.gradient(self.fx)
                case "y":
                    self.dy = np.gradient(self.fy)
                case "z":
                    self.dz = np.gradient(self.fz)

    def fetch_plate_data(self):
        """Fetch plate data using Vicon SDK"""
        device_ids = self.vicon.vicon.GetDeviceIDs()
        for device_id in device_ids:
            details = self.vicon.vicon.GetDeviceDetails(device_id)
            if details[1] == "ForcePlate":
                wt = details[4].WorldT
                plate_name = plate_configs.get(tuple(wt))  # type: ignore
                if plate_name == self.name:
                    self.wt = wt
                    self.name = plate_name
                    self.wr = details[4].WorldR
                    self.fx = self.vicon.vicon.GetDeviceChannel(device_id, 1, 1)[0]
                    self.fy = self.vicon.vicon.GetDeviceChannel(device_id, 1, 2)[0]
                    self.fz = self.vicon.vicon.GetDeviceChannel(device_id, 1, 3)[0]
                    self.copx = self.vicon.vicon.GetDeviceChannel(device_id, 3, 1)[0]
                    self.copy = self.vicon.vicon.GetDeviceChannel(device_id, 3, 2)[0]
                    self.copz = self.vicon.vicon.GetDeviceChannel(device_id, 3, 3)[0]
                    self.copx_raw = copy.deepcopy(self.copx)
                    self.copy_raw = copy.deepcopy(self.copy)
                    self.copz_raw = copy.deepcopy(self.copz)
                    self.mx = self.vicon.vicon.GetDeviceChannel(device_id, 2, 1)[0]
                    self.my = self.vicon.vicon.GetDeviceChannel(device_id, 2, 2)[0]
                    self.mz = self.vicon.vicon.GetDeviceChannel(device_id, 2, 3)[0]
                    for idx, (a, b) in enumerate(zip(self.copx, self.copy)):
                        self.copx[idx] = self.wt[0] - b
                        self.copy[idx] = self.wt[1] + a
                    strike = self.find_plate_strikes()
                    return strike

    def find_plate_strikes(self):
        """Find the frame intervals when a foot is exerting force on a plate"""
        lower, upper = self.vicon.get_region_of_interest()
        strike_intervals = []
        possible_strike = False
        fc = 0
        start_frame = end_frame = None
        # 2000hz refresh for plates, 200hz for camera
        for i in range(lower * 10, upper * 10):
            resultant = math.sqrt(self.fx[i] ** 2 + self.fy[i] ** 2 + self.fz[i] ** 2)
            if resultant > 15:
                if not possible_strike:
                    possible_strike = True
                    start_frame = i
                elif fc > 15:
                    end_frame = i
                fc += 1
            else:
                if start_frame and end_frame:
                    strike_intervals.append((start_frame, end_frame))
                start_frame = None
                end_frame = None
                fc = 0
                possible_strike = False
        return strike_intervals

    def find_plate_matches(self, strike_intervals):
        """Match strike intervals on a plate to a foot"""

        results = {"left": [], "right": []}
        if len(strike_intervals) == 0:
            return results

        def is_intersecting(box1, box2):
            min_x1, max_x1, min_y1, max_y1, _ = box1
            min_x2, max_x2, min_y2, max_y2 = box2
            x_overlap = not (max_x1 < min_x2 or max_x2 < min_x1)
            y_overlap = not (max_y1 < min_y2 or max_y2 < min_y1)
            return x_overlap and y_overlap

        def frame_in_strike_interval(j):
            for intervals in strike_intervals:
                start = intervals[0] // 10
                end = intervals[1] // 10
                if start <= j <= end:
                    return True
            return False

        plate_bounds = [self.wt[0] - 300, self.wt[0] + 300, self.wt[1] - 300, self.wt[1] + 300]  # type: ignore
        for foot in self.vicon.strike_events:
            for i in range(len(self.vicon.off_events[foot])):
                for j in range(
                    self.vicon.strike_events[foot][i],
                    (self.vicon.off_events[foot][i] + 1),
                ):
                    bbox = (
                        right_foot.calculate_bounding_box(j)
                        if foot == "right"
                        else left_foot.calculate_bounding_box(j)
                    )
                    min_x, max_x, min_y, max_y, min_z = bbox
                    # foot not in bounds of the plates
                    if (
                        2712 < min_x
                        and 300 > max_x
                        and 903 < min_y
                        and 0 > max_y
                        or not frame_in_strike_interval(j)
                    ):
                        continue
                    if foot == "left":
                        if (
                            is_intersecting(bbox, plate_bounds)
                            and frame_in_strike_interval(j)
                            and left_foot.is_strike_in_plate(
                                self.copx[j * 10 - 10],
                                self.copy[j * 10 - 10],
                                min_z,
                                j,
                            )
                        ):
                            results[foot].append(j)
                    else:
                        if (
                            is_intersecting(bbox, plate_bounds)
                            and frame_in_strike_interval(j)
                            and right_foot.is_strike_in_plate(
                                self.copx[j * 10 - 10],
                                self.copy[j * 10 - 10],
                                min_z,
                                j,
                            )
                        ):
                            results[foot].append(j)
        results = self.format_results(results)
        return results

    def format_results(self, results):
        """Format results from plate matches to show intervals instead of individual frames"""
        left_intervals = []
        right_intervals = []
        left = results["left"]
        right = results["right"]
        if left:
            start = left[0]
            end = left[0]
            for i in range(1, len(left)):
                if left[i] == end + 1 or left[i] == end + 2:
                    end = left[i]
                else:
                    left_intervals.append((start, end))
                    start = left[i]
                    end = left[i]
            left_intervals.append((start, end))
            results["left"] = left_intervals
        if right:
            start = right[0]
            end = right[0]
            for i in range(1, len(right)):
                if right[i] == end + 1 or right[i] == end + 2:
                    end = right[i]
                else:
                    right_intervals.append((start, end))
                    start = right[i]
                    end = right[i]
            right_intervals.append((start, end))
            results["right"] = right_intervals
        return results

    def plot_forces(self):
        """Plot forces over time"""
        plt.figure(figsize=(10, 6))
        frames = np.arange(0, len(self.fx))
        self.calculate_gradient("x")
        self.calculate_gradient("y")
        self.calculate_gradient("z")
        plt.plot(frames, self.fx, label="x force")
        plt.plot(frames, self.fy, label="y force")
        plt.plot(frames, self.fz, label="z force")
        plt.plot(frames, self.dx, label="dx")
        plt.plot(frames, self.dy, label="dy")
        plt.plot(frames, self.dz, label="dz")

        plt.xlabel("Frame")
        plt.title("Force over time")
        plt.legend()
        plt.grid(True)
        plt.show()


# pylint: disable=missing-function-docstring
def driver():
    vicon = Vicon()
    plates = [
        "Plate1",
        "Plate2",
        "Plate3",
        "Plate4",
        "Plate5",
        "Plate6",
        "Plate7",
        "Plate8",
        "Plate9",
    ]
    plate_objs = []
    results = {plate: {} for plate in plates}
    for plate in plates:
        print(plate)
        p = Plate(plate, vicon)
        plate_objs.append(p)
        intervals = p.fetch_plate_data()
        print(intervals)
        results[plate] = p.find_plate_matches(intervals)
    print(results)
    return results, plate_objs


if __name__ == "__main__":
    driver()
