import matplotlib.pyplot as plt
import numpy as np

def triangleMF(x, a, b, c, LB=0.0, UB=0.0):
    if x < a:
        return LB
    elif a <= x < b:
        return (x - a) / (b - a)
    elif b <= x < c:
        return (c - x) / (c - b)
    else:
        return UB

class VelocityMembership:
    def __init__(self):
        # Labels as in the image and your description
        self.labels = [
            'TBW', 'VFSBW'
            , 'FSBW', 'BW', 'SLBW', 'VSLBW', 'S',
            'VSLFW', 'SLFW', 'FW', 'FSFW', 'VFSFW', 'TFW'
        ]
        self.abc = [[-400, -350, -300],  # TBW
              [-350, -300, -250],   # VFSBW
              [-300, -250, -200],   # FSBW
              [-250, -200, -150],   # BW
              [-200, -150, -100],   # SLBW
              [-150, -100, -50],    # VSLBW
              [-100, 0, 100],       # S
              [50, 100, 150],       # VSLFW
              [100, 150, 200],       # SLFW
              [150, 200, 250],      # FW
              [200, 250, 300],      # FSFW
              [250, 300, 350],      # VFSFW
              [300, 350, 400]]      # TFW

    def evaluate(self, x):
        memberships = {}
        for i, label in enumerate(self.labels):
            UB = 0
            LB = 0
            a = self.abc[i][0]
            b = self.abc[i][1]
            c = self.abc[i][2]
            memberships[label] = triangleMF(x, a, b, c, LB, UB)
        return memberships

    def plot(self, x_vals):
        plt.figure(figsize=(12, 6))
        for i, label in enumerate(self.labels):
            y_vals = [self.evaluate(x)[label] for x in x_vals]
            plt.plot(x_vals, y_vals, label=label)
        plt.title('Velocity Fuzzy Membership Functions')
        plt.xlabel(r'$v_x, v_y$ (mm/sec)')
        plt.ylabel('Membership Degree')
        plt.grid(True)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5)
        plt.tight_layout()
        plt.show()

class DistanceMembership:
    def __init__(self):
        # Labels as in the image: Very Close Left, Close Left, Middle, Far, Very Far, Infinite
        self.labels = ['Z', 'VCL', 'CL', 'M', 'F', 'VF', 'IN']
        # Centers based on the image (approximate, adjust as needed)
        self.centers = np.array([0, 50, 100, 150, 200, 250, 300])

    def evaluate(self, x):
        memberships = {}
        for i, label in enumerate(self.labels):
            UB = 0
            LB = 0
            if i == 0:
                LB = 1.0
            elif i == len(self.labels) - 1:
                UB = 1.0
            a = self.centers[i - 1] if i > 0 else self.centers[i]
            b = self.centers[i]
            c = self.centers[i + 1] if i < len(self.centers) - 1 else self.centers[i]
            memberships[label] = triangleMF(x, a, b, c, LB, UB)
        return memberships

    def plot(self, x_vals):
        plt.figure(figsize=(8, 4))
        for i, label in enumerate(self.labels):
            UB = 0
            LB = 0
            if i == 0:
                LB = 1.0
            elif i == len(self.labels) - 1:
                UB = 1.0
            a = self.centers[i - 1] if i > 0 else self.centers[i]
            b = self.centers[i]
            c = self.centers[i + 1] if i < len(self.centers) - 1 else self.centers[i]
            y_vals = [triangleMF(x, a, b, c, LB, UB) for x in x_vals]
            plt.plot(x_vals, y_vals, label=label)
        plt.title('Distance Fuzzy Membership Functions')
        plt.xlabel(r'$d_e$ (cm)')
        plt.ylabel('Membership Degree')
        plt.grid(True)
        plt.legend(loc='upper right')
        plt.tight_layout()
        plt.show()

class DegreeMembership:
    def __init__(self):
        self.labels = ['NIN', 'NVL', 'NL', 'NM', 'NS', 'NVS', 'ZE', 'PVS', 'PS', 'PM', 'PL', 'PVL', 'PIN']
        self.centers = np.linspace(-180, 180, 13)

    def evaluate(self, x):
        memberships = {}
        for i, label in enumerate(self.labels):
            UB = 0
            LB = 0
            if i == 0:
                LB = 1.0
            elif i == len(self.labels) - 1:
                UB = 1.0
            a = self.centers[i - 1] if i > 0 else self.centers[i]
            b = self.centers[i]
            c = self.centers[i + 1] if i < len(self.centers) - 1 else self.centers[i]
            memberships[label] = triangleMF(x, a, b, c, LB, UB)
        return memberships

    def plot(self, x_vals):
        plt.figure(figsize=(12, 6))
        for i, label in enumerate(self.labels):
            UB = 0
            LB = 0
            if i == 0:
                LB = 1.0
            elif i == len(self.labels) - 1:
                UB = 1.0
            a = self.centers[i - 1] if i > 0 else self.centers[i]
            b = self.centers[i]
            c = self.centers[i + 1] if i < len(self.centers) - 1 else self.centers[i]
            y_vals = [triangleMF(x, a, b, c, LB, UB) for x in x_vals]
            plt.plot(x_vals, y_vals, label=label)
        plt.title('Fuzzy Membership Functions')
        plt.xlabel('de (degrees)')
        plt.ylabel('Membership Degree')
        plt.grid(True)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5)
        plt.tight_layout()
        plt.show()

vx_rule_table = [
    #   Z      VCL      CL      M      F      VF     IN
    ['S',  'VSLBW', 'SLBW',  'BW',  'FSBW', 'VFSBW', 'TBW'],   # NIN
    ['S',      'BW', 'FSBW', 'FSBW','FSBW', 'VFSBW', 'TBW'],   # NVL
    ['S',  'VSLBW', 'SLBW',  'BW',  'FSBW', 'FSBW',  'VFSBW'], # NL
    ['S',       'S',     'S',   'S',     'S',   'S',      'S'], # NM
    ['S', 'VSLFW', 'SLFW',  'FW',  'FSFW', 'VFSFW', 'TFW'],    # NS
    ['S',     'FW', 'FSFW', 'FSFW','FSFW', 'VFSFW', 'TFW'],    # NVS
    ['S', 'VSLFW', 'SLFW',  'FW',  'FSFW', 'FSFW',  'VFSFW'],  # ZE
    ['S',     'FW', 'FSFW', 'FSFW','FSFW', 'VFSFW', 'TFW'],    # PVS
    ['S', 'VSLFW', 'SLFW',  'FW',  'FSFW', 'FSFW',  'VFSFW'],  # PS
    ['S',       'S',     'S',   'S',     'S',   'S',      'S'], # PM
    ['S',  'VSLBW', 'SLBW',  'BW',  'FSBW', 'FSBW',  'VFSBW'], # PL
    ['S',      'BW', 'FSBW', 'FSBW','FSBW', 'VFSBW', 'TBW'],   # PVL
    ['S',  'VSLBW', 'SLBW',  'BW',  'FSBW', 'VFSBW', 'TBW'],   # PIN
]

vy_rule_table = [
    #     Z      VCL      CL      M      F      VF     IN
    ['S',    'S',    'S',    'S',    'S',    'S',    'S'],        # NIN
    ['S', 'VSLFW','VSLFW','SLFW','SLFW',   'FW', 'FSFW'],         # NVL
    ['S', 'SLFW',   'FW', 'FSFW','VFSFW','TFW', 'TFW'],           # NL
    ['S', 'VSLFW','SLFW',   'FW', 'FSFW','VFSFW','TFW'],          # NM
    ['S', 'SLFW',   'FW', 'FSFW','VFSFW','TFW', 'TFW'],           # NS
    ['S', 'VSLFW','SLFW',   'FW', 'FSFW','VFSFW','TFW'],          # NVS
    ['S',    'S',    'S',    'S',    'S',    'S',    'S'],        # ZE
    ['S', 'VSLBW','VSLBW','SLBW',   'BW', 'FSBW','FSBW'],         # PVS
    ['S', 'SLBW',   'BW', 'FSBW','VFSBW','TFBW','TFBW'],          # PS
    ['S', 'VSLBW','SLBW',   'BW', 'FSBW','VFSBW','TFBW'],         # PM
    ['S', 'SLBW',   'BW', 'FSBW','VFSBW','TFBW','TFBW'],          # PL
    ['S', 'VSLBW','SLBW',   'BW', 'FSBW','VFSBW','FSBW'],         # PVL
    ['S',    'S',    'S',    'S',    'S',    'S',    'S'],        # PIN
]

# Usage:
# theta_idx = DegreeMembership().labels.index(theta_label)
# dist_idx = DistanceMembership().labels.index(dist_label)
# velocity_label = fuzzy_rule_table_2[theta_idx][dist_idx]

# Usage:
# theta_idx = DegreeMembership().labels.index(theta_label)
# dist_idx = DistanceMembership().labels.index(dist_label)
# velocity_label = fuzzy_rule_table[theta_idx][dist_idx]

# th_vals = np.linspace(-200, 200, 1000)
thm = DegreeMembership()
# thm.plot(th_vals)

# x_vals = np.linspace(0, 320, 1000)
dem = DistanceMembership()
# dem.plot(x_vals)

v_vals = np.linspace(-420, 420, 1000)
vm = VelocityMembership()
vm.plot(v_vals)