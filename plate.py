import numpy as np

class Plate:
    name = ''
    fx = fy = fz = []
    dx = dy = dz = []

    def __init__(self, name, fx, fy, fz):
        self.name = name
        self.fx = fx
        self.fy = fy
        self.fz = fz

    def __str__(self):
        return self.name

    def calculate_gradient(self, dimension = 'x'):
        match dimension:
            case 'x':
                self.dx = np.gradient(self.fx)
            case 'y':
                self.dy = np.gradient(self.fy)
            case 'z':
                self.dz = np.gradient(self.fz)