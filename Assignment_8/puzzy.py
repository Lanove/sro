from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# import keyboard
import time
import numpy as np
import matplotlib.pyplot as plt


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


        self.labels = [
            'TBW', 'VFSBW'
            , 'FSBW', 'BW', 'SLBW', 'VSLBW', 'S',
            'VSLFW', 'SLFW', 'FW', 'FSFW', 'VFSFW', 'TFW'
        ]
        
vx_rule_table = [
    #   Z      VCL      CL      M      F      VF     IN
    ['S',  'VSLBW', 'SLBW',  'BW',  'FSBW', 'VFSBW', 'TBW'],   # NIN
    ['S',      'BW', 'BW', 'FSBW','FSBW', 'VFSBW', 'TBW'],   # NVL
    ['S',  'VSLBW', 'SLBW',  'BW',  'BW', 'FSBW',  'VFSBW'], # NL
    ['S',       'S',     'S',   'S',     'S',   'S',      'S'], # NM
    ['S', 'VSLFW', 'SLFW',  'FW',  'FW', 'FSFW', 'VFSFW'],    # NS
    ['S',     'FW', 'FW', 'FSFW','FSFW', 'VFSFW', 'TFW'],    # NVS
    ['S', 'VSLFW', 'SLFW',  'FW',  'FSFW', 'VFSFW',  'TFW'],  # ZE
    ['S',     'FW', 'FW', 'FSFW','FSFW', 'VFSFW', 'TFW'],    # PVS
    ['S', 'VSLFW', 'SLFW',  'FW',  'FW', 'FSFW',  'VFSFW'],  # PS
    ['S',       'S',     'S',   'S',     'S',   'S',      'S'], # PM
    ['S',  'VSLBW', 'SLBW',  'BW',  'BW', 'FSBW',  'VFSBW'], # PL
    ['S',      'BW', 'BW', 'FSBW','FSBW', 'VFSBW', 'TBW'],   # PVL
    ['S',  'VSLBW', 'SLBW',  'BW',  'FSBW', 'VFSBW', 'TBW'],   # PIN
]

vy_rule_table = [
    #     Z      VCL      CL      M      F      VF     IN
    ['S',    'S',    'S',    'S',    'S',    'S',    'S'],        # NIN
    ['S', 'VSLFW','VSLFW','SLFW','SLFW',   'FW', 'FSFW'],         # NVL
    ['S', 'SLFW',   'FW', 'FSFW','FSFW','VFSFW', 'TFW'],           # NL
    ['S', 'VSLFW','SLFW',   'FW', 'FSFW','VFSFW','TFW'],          # NM
    ['S', 'SLFW',   'FW', 'FSFW','FSFW','VFSFW', 'TFW'],           # NS
    ['S', 'VSLFW','VSLFW',   'SLFW', 'SLFW','FW','FSFW'],          # NVS
    ['S',    'S',    'S',    'S',    'S',    'S',    'S'],        # ZE
    ['S', 'VSLBW','VSLBW','SLBW',   'SLBW', 'BW','FSBW'],         # PVS
    ['S', 'SLBW',   'BW', 'FSBW','FSBW','VFSBW','TBW'],          # PS
    ['S', 'VSLBW','SLBW',   'BW', 'FSBW','VFSBW','TBW'],         # PM
    ['S', 'SLBW',   'BW', 'FSBW','FSBW','VFSBW','TBW'],          # PL
    ['S', 'VSLBW','VSLBW',   'SLBW', 'SLBW','BW','FSBW'],         # PVL
    ['S',    'S',    'S',    'S',    'S',    'S',    'S'],        # PIN
]

def getTargetPose():
    # Get the target handle
  targetHandle = sim.getObject("/Target")  # Use the name you gave it

  # Get target position
  targetPosition = sim.getObjectPosition(targetHandle, -1)
  targetOrientation = sim.getObjectOrientation(targetHandle, -1)

  target_x, target_y = targetPosition[0], targetPosition[1]
  target_yaw = targetOrientation[2]

  return target_x, target_y, target_yaw

def getRobotPose():
    # Get the robot handle
    robotHandle = sim.getObject("/PioneerP3DX")  # Adjust path if needed
    
    # Get position (x, y, z)
    position = sim.getObjectPosition(robotHandle, -1)  # -1 means relative to world frame
    
    # Get orientation (alpha, beta, gamma) - in Euler angles
    orientation = sim.getObjectOrientation(robotHandle, -1)
    
    # Extract x, y, and yaw
    x, y = position[0], position[1]
    yaw = orientation[2]  # gamma component is the yaw
    
    return x, y, yaw

def getMotorsHandle():
    motorRightHandle = sim.getObject("/PioneerP3DX/rightMotor")
    motorLeftHandle = sim.getObject("/PioneerP3DX/leftMotor")
    return (motorRightHandle, motorLeftHandle)

def setRobotMotion(motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])
    return

def getErrorPose(pose_des, pose_act):
    # Calculate the error in position and orientation
    ex = pose_des[0] - pose_act[0]
    ey = pose_des[1] - pose_act[1]
    eyaw = pose_des[2] - pose_act[2]
    
    return ex, ey, eyaw

def inverseKinematics(cmd_bcs):
    inverse_matrix = np.linalg.inv(np.array([[R/2, R/2], [R/(2*L), -R/(2*L)]]))
    return np.dot(inverse_matrix, cmd_bcs)

def cmd_wcs2bcs(vd, theta):
    tf_mat = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])
    tf_mat_pinv = np.linalg.pinv(tf_mat)
    return np.dot(tf_mat_pinv, vd)
    
client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(False)
sim.startSimulation()
motors_handle = getMotorsHandle()
ed_epsilon = 0.05
R = 0.0975 # radius of the wheel
L = 0.381 # distance between the wheels
phi_normal = 5

dem = DistanceMembership()
thm = DegreeMembership()
vm = VelocityMembership()

while True:
    current_time = time.time()
    target_pose = getTargetPose()
    current_pose = getRobotPose()
    ex, ey, eyaw = getErrorPose(target_pose, current_pose)
    
    ex *= 100.
    ey *= 100.
    
    ed = np.sqrt(ex**2 + ey**2)
    
    eyaw = np.degrees(eyaw)
    if eyaw > 180:
        eyaw -= 360
    elif eyaw < -180:
        eyaw += 360
        
    dem.evaluate(ed)
    thm.evaluate(eyaw)
    
    fuzzy_d = dem.evaluate(ed)
    fuzzy_t = thm.evaluate(eyaw)
    
    numvx, numvy = 0, 0
    denvx, denvy = 0, 0

    for d_label, d_val in fuzzy_d.items():
        for t_label, t_val in fuzzy_t.items():
            d_idx = dem.labels.index(d_label)
            t_idx = thm.labels.index(t_label)

            vx_out = vx_rule_table[t_idx][d_idx]
            vy_out = vy_rule_table[t_idx][d_idx]

            rule_strength = min(d_val, t_val)
            
            vx_idx = vm.labels.index(vx_out)
            centroid = vm.abc[vx_idx][1]
            numvx += rule_strength * centroid
            denvx += rule_strength
            
            vy_idx = vm.labels.index(vy_out)
            centroid = vm.abc[vy_idx][1]  
            numvy += rule_strength * centroid
            denvy += rule_strength

    vx = numvx/denvx if denvx > 0 else 0
    vy = numvy/denvy if denvy > 0 else 0
    
    vx /= 1000. 
    vy /= 1000.
    
    theta = np.arctan2(ey, ex)
    yawd_des = theta - current_pose[2]
    
    cmd_wcs = np.array([vx, vy, yawd_des])
    cmd_bcs = cmd_wcs2bcs(cmd_wcs, theta)
    phi_dot_vector = inverseKinematics(cmd_bcs)
    
    phi_max = np.max(phi_dot_vector)
    phi_rn = phi_dot_vector[0]
    phi_ln = phi_dot_vector[1]
    
    if phi_max > phi_normal:
        phi_rn = phi_normal * phi_rn / phi_max
        phi_ln = phi_normal * phi_ln / phi_max

    wheel_vel = [phi_rn, phi_ln]
    
    setRobotMotion(motors_handle, wheel_vel)