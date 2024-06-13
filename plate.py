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