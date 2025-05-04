import numpy as np
import matplotlib.pyplot as plt

class FuzzyMembership:
    def __init__(self, labels, centers_or_abc):
        self.labels = labels
        self.is_triangles = isinstance(centers_or_abc[0], (list, tuple, np.ndarray))
        if self.is_triangles:
            self.abc = centers_or_abc
        else:
            self.centers = np.array(centers_or_abc)

    def triangleMF(self, x, a, b, c, LB=0.0, UB=0.0):
        if x < a:
            return LB
        elif a <= x < b:
            return (x - a) / (b - a)
        elif b <= x < c:
            return (c - x) / (c - b)
        else:
            return UB

    def evaluate(self, x):
        memberships = {}
        for i, label in enumerate(self.labels):
            LB = 1.0 if i == 0 else 0
            UB = 1.0 if i == len(self.labels) - 1 else 0
            if self.is_triangles:
                a, b, c = self.abc[i]
            else:
                a = self.centers[i - 1] if i > 0 else self.centers[i]
                b = self.centers[i]
                c = self.centers[i + 1] if i < len(self.centers) - 1 else self.centers[i]
            memberships[label] = self.triangleMF(x, a, b, c, LB, UB)
        return memberships

    def plot(self, x_vals, xlabel='x', title='Fuzzy Membership Functions'):
        plt.figure(figsize=(12, 6))
        for i, label in enumerate(self.labels):
            LB = 1.0 if i == 0 else 0
            UB = 1.0 if i == len(self.labels) - 1 else 0
            if self.is_triangles:
                a, b, c = self.abc[i]
            else:
                a = self.centers[i - 1] if i > 0 else self.centers[i]
                b = self.centers[i]
                c = self.centers[i + 1] if i < len(self.centers) - 1 else self.centers[i]
            y_vals = [self.triangleMF(x, a, b, c, LB, UB) for x in x_vals]
            plt.plot(x_vals, y_vals, label=label)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel('Membership Degree')
        plt.grid(True)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5)
        plt.tight_layout()
        plt.show()

# Velocity
vel_labels = [
    'TBW', 'VFSBW', 'FSBW', 'BW', 'SLBW', 'VSLBW', 'S',
    'VSLFW', 'SLFW', 'FW', 'FSFW', 'VFSFW', 'TFW'
]
vel_abc = [
    [-400, -350, -300], [-350, -300, -250], [-300, -250, -200], [-250, -200, -150],
    [-200, -150, -100], [-150, -100, -50], [-100, 0, 100], [50, 100, 150],
    [100, 150, 200], [150, 200, 250], [200, 250, 300], [250, 300, 350], [300, 350, 400]
]
velocity_mf = FuzzyMembership(vel_labels, vel_abc)

# Distance
dist_labels = ['Z', 'VCL', 'CL', 'M', 'F', 'VF', 'IN']
dist_centers = [0, 50, 100, 150, 200, 250, 300]
distance_mf = FuzzyMembership(dist_labels, dist_centers)
d_vals = np.linspace(0, 300, 100)
distance_mf.plot(d_vals, xlabel='Distance (m)', title='Distance Membership Functions')

# Degree
deg_labels = ['NIN', 'NVL', 'NL', 'NM', 'NS', 'NVS', 'ZE', 'PVS', 'PS', 'PM', 'PL', 'PVL', 'PIN']
deg_centers = np.linspace(-180, 180, 13)
degree_mf = FuzzyMembership(deg_labels, deg_centers)
deg_vals = np.linspace(-180, 180, 1000)
degree_mf.plot(deg_vals, xlabel='de (degrees)', title='Degree Membership Functions')

# Plot example
x_vals = np.linspace(-400, 400, 1000)
velocity_mf.plot(x_vals, xlabel='$v_x, v_y$ (mm/sec)', title='Velocity Fuzzy Membership Functions')