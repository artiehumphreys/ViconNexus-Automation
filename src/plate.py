import numpy as np
import sys
import os
from vicon import Vicon

plate_configs = {
    [2712.0,300.0,0.0] : 'Plate1',
    [2712.0,903.0,0.0] : 'Plate2', 
    [2109.0,300.0,0.0] : 'Plate3', 
    [2109.0,903.0,0.0] : 'Plate4',
    [1506.0,300.0,0.0] : 'Plate5',
    [1506.0,903.0,0.0] : 'Plate6',
    [903.0,300.0,0.0] : 'Plate7',
    [903.0,903.0,0.0] : 'Plate8',
    [300.0,300.0,0.0] : 'Plate9',
}

class Plate:
    def __init__ (self, name):
        self.name = name
        self.vicon = Vicon()
        self.fx = fx
        self.fy = fy
        self.fz = fz
        self.dx = []
        self.dy = []
        self.dz = []
        self.copx = []
        self.copy = []
        self.copz = []
        self.mx = []
        self.my = []
        self.mz = []

    def __str__(self):
        return self.name

    def calculate_gradient(self, dimension = 'x'):
        if fx and fy and fz:
            match dimension:
                case 'x':
                    self.dx = np.gradient(self.fx)
                case 'y':
                    self.dy = np.gradient(self.fy)
                case 'z':
                    self.dz = np.gradient(self.fz)

    def find_plate_data(self):
        strikes = []
        deviceIDs = self.vicon.GetDeviceIDs()
        for deviceID in deviceIDs:
            details = vicon.GetDeviceDetails(deviceID)
            if details['type'] == 'ForcePlate':
                wt = details['force_plate']['WorldT']
                plate_name = plate_configs.get(tuple(wt))
                if plate_name:
                    self.name = plate_name
                    wr = details['force_plate']['WorldR']
                    self.fx = vicon.GetDeviceChannel(deviceID, 1, 1)[0]
                    self.fy = vicon.GetDeviceChannel(deviceID, 1, 2)[0]
                    self.fz = vicon.GetDeviceChannel(deviceID, 1, 3)[0]
                    self.copx = vicon.GetDeviceChannel(deviceID, 3, 1)[0]
                    self.copy = vicon.GetDeviceChannel(deviceID, 3, 2)[0]
                    self.copz = vicon.GetDeviceChannel(deviceID, 3, 3)[0]
                    self.mx = vicon.GetDeviceChannel(deviceID, 2, 1)[0]
                    self.my = vicon.GetDeviceChannel(deviceID, 2, 2)[0]
                    self.mz = vicon.GetDeviceChannel(deviceID, 2, 3)[0]
                    for x in range(len(copx)):
                        a = self.copx[x]
                        b = self.copy[x]
                        self.copx[x] = self.wt[0] - b
                        self.copy[x] = self.wt[1] + a

                strike = self.find_plate_strikes()
                if strike:
                    strikes.append((strike, name))
        return strikes

    def find_plate_strikes(self):
        strike_intervals = []
        possible_strike = False
        fc = 0
        start_frame = end_frame = None
        # 2000hz refresh for plates, 200hz for camera
        for i in range(self.vicon.get_region_of_interest[0] * 10, self.vicon.get_region_of_interest[1] * 10):
            resultant =  math.sqrt(self.fx[i] ** 2 + self.fy[i] ** 2 + self.fz[i] ** 2)
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

    def plot_forces(self):
        plt.figure(figsize=(10,6))
        frames = np.arange(0, len(self.fx)) 
        self.calculate_gradient('x')
        self.calculate_gradient('y')
        self.calculate_gradient('z')
        plt.plot(frames, self.fx, label = 'x force') 
        plt.plot(frames, self.fy, label = 'y force') 
        plt.plot(frames, self.fz, label = 'z force') 
        plt.plot(frames, self.dx, label = 'dx') 
        plt.plot(frames, self.dy, label = 'dy') 
        plt.plot(frames, self.dz, label = 'dz') 

        plt.xlabel('Frame')
        plt.title('Force over time')
        plt.legend()
        plt.grid(True)
        plt.show()

def main():
    return

if __name__ == "__main__":
    main()