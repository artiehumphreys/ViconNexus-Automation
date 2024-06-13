import numpy as np
import sys
import os
from vicon import Vicon

class Plate:
    name = ''
    fx = fy = fz = []
    dx = dy = dz = []

    def __init__(self, name, fx, fy, fz):
        self.name = name
        self.vicon = Vicon()

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
                _, type, rate, output, force_plate, channel = vicon.GetDeviceDetails(deviceID)
                if type == 'ForcePlate':
                    wt = force_plate.WorldT
                    wr = force_plate.WorldR
                    fx = vicon.GetDeviceChannel(deviceID, 1, 1)[0]
                    fy = vicon.GetDeviceChannel(deviceID, 1, 2)[0]
                    fz = vicon.GetDeviceChannel(deviceID, 1, 3)[0]
                    copx = vicon.GetDeviceChannel(deviceID, 3, 1)[0]
                    copy = vicon.GetDeviceChannel(deviceID, 3, 2)[0]
                    copz = vicon.GetDeviceChannel(deviceID, 3, 3)[0]
                    mx = vicon.GetDeviceChannel(deviceID, 2, 1)[0]
                    my = vicon.GetDeviceChannel(deviceID, 2, 2)[0]
                    mz = vicon.GetDeviceChannel(deviceID, 2, 3)[0]
                    name = find_plate_name(wt)
                    for x in range(len(copx)):
                        a = copx[x]
                        b = copy[x]
                        copx[x] = wt[0] - b
                        copy[x] = wt[1] + a
                    if name:
                        fp_data[name] = {
                            'fx': fx,
                            'fy': fy,
                            'fz': fz,
                            'copx': copx,
                            'copy': copy,
                            'copz': copz,
                            'mx': mx,
                            'my': my,
                            'mz': mz,
                            'wr': wr,
                            'wt': wt
                        }
                    fp = Plate(name, fx, fy, fz)
                    strike = find_plate_strikes(fp)
                    if strike:
                        strikes.append((strike, name))
        return strikes

    def find_plate_strikes(fp):
        strike_intervals = []
        possible_strike = False
        fc = 0
        start_frame = end_frame = None
        for i in range(user_defined_region[0] * 10, user_defined_region[1] * 10):
            resultant =  math.sqrt(fp.fx[i] ** 2 + fp.fy[i] ** 2 + fp.fz[i] ** 2)
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