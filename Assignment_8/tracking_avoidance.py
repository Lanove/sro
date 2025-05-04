from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# import keyboard
import time
import numpy as np
import matplotlib.pyplot as plt

R = 0.0975
L = 0.381

def triangleMF(x, a, b, c, LB=0.0, UB=0.0):
    if x < a:
        return LB
    elif a <= x < b:
        return (x - a) / (b - a)
    elif b <= x < c:
        return (c - x) / (c - b)
    else:
        return UB

def getDistances(sim, sensorsHandle):
    Distances = []
    for i in range(16):
        detectionState, _, detectedPoint, _, _ = sim.readProximitySensor(sensorsHandle[i])
        distanceValue = detectedPoint[2]
        if detectionState == False:
            distanceValue = 1.0
        Distances.append(distanceValue)
    return Distances

def getTargetPose():
  targetHandle = sim.getObject("/Target") 

  targetPosition = sim.getObjectPosition(targetHandle, -1)
  targetOrientation = sim.getObjectOrientation(targetHandle, -1)

  target_x, target_y = targetPosition[0], targetPosition[1]
  target_yaw = targetOrientation[2]

  return target_x, target_y, target_yaw

def getRobotPose():
    robotHandle = sim.getObject("/PioneerP3DX")  
    
    position = sim.getObjectPosition(robotHandle, -1) 
    
    orientation = sim.getObjectOrientation(robotHandle, -1)
    
    x, y = position[0], position[1]
    yaw = orientation[2]  
    
    return x, y, yaw

def getSensorsHandle(sim):
    sensorsHandle = []
    for i in range(16):
        sensorHandle = sim.getObject('/PioneerP3DX/ultrasonicSensor[' + str(i) + ']')
        sensorsHandle.append(sensorHandle)
    _, _, _, _, _ = sim.handleProximitySensor(sim.handle_all)
    return sensorsHandle

def getMotorsHandle():
    motorRightHandle = sim.getObject("/PioneerP3DX/rightMotor")
    motorLeftHandle = sim.getObject("/PioneerP3DX/leftMotor")
    return (motorRightHandle, motorLeftHandle)

def setRobotMotion(motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])
    return

def getErrorPose(pose_des, pose_act):
    ex = pose_des[0] - pose_act[0]
    ey = pose_des[1] - pose_act[1]
    eyaw = pose_des[2] - pose_act[2]
    
    return ex, ey, eyaw

def forwardKinematics(q):
    vx = R/2 * (q[0] + q[1])
    w = R/(2*L) * (q[0] - q[1])
    return np.array([vx, w])

def inverseKinematics(v_bcs):
    inverse_matrix = np.linalg.inv(np.array([[R/2, R/2], [R/(2*L), -R/(2*L)]]))
    return np.dot(inverse_matrix, v_bcs)

def cmd_wcs2bcs(vd, theta):
    tf_mat = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])
    tf_mat_pinv = np.linalg.pinv(tf_mat)
    return np.dot(tf_mat_pinv, vd)

# Add this class before your main loop
class PositionTracker:
    def __init__(self, max_points=100):
        # Setup figure and plots
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Robot and Target Tracking')
        self.ax.grid(True)
        
        # Initialize data storage
        self.max_points = max_points
        self.robot_x = []
        self.robot_y = []
        self.target_x = [] 
        self.target_y = []
        
        # Plot elements
        self.robot_trail, = self.ax.plot([], [], 'b-', alpha=0.5, label='Robot Path')
        self.target_trail, = self.ax.plot([], [], 'r-', alpha=0.5, label='Target Path')
        self.robot_pos = self.ax.scatter([], [], color='blue', s=100, label='Robot')
        self.target_pos = self.ax.scatter([], [], color='red', s=100, label='Target')
        
        # Robot and target orientation arrows
        self.robot_arrow = self.ax.quiver([], [], [], [], color='blue', scale=20)
        self.target_arrow = self.ax.quiver([], [], [], [], color='red', scale=20)
        
        # Error display text
        self.error_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        
        self.ax.legend()
        plt.show(block=False)
        self.fig.canvas.draw()
    
    def update(self, target_pose, robot_pose, ed, eyaw):
        # Append new data (limited to max_points)
        self.robot_x.append(robot_pose[0])
        self.robot_y.append(robot_pose[1])
        self.target_x.append(target_pose[0])
        self.target_y.append(target_pose[1])
        
        # Trim lists to max_points
        if len(self.robot_x) > self.max_points:
            self.robot_x.pop(0)
            self.robot_y.pop(0)
            self.target_x.pop(0)
            self.target_y.pop(0)
        
        # Update trails
        self.robot_trail.set_data(self.robot_x, self.robot_y)
        self.target_trail.set_data(self.target_x, self.target_y)
        
        # Update current positions
        self.robot_pos.set_offsets(np.column_stack([robot_pose[0], robot_pose[1]]))
        self.target_pos.set_offsets(np.column_stack([target_pose[0], target_pose[1]]))
        
        # Update direction arrows
        robot_dx = 0.3 * np.cos(robot_pose[2])
        robot_dy = 0.3 * np.sin(robot_pose[2])
        self.robot_arrow.set_offsets(np.array([robot_pose[0], robot_pose[1]]))
        self.robot_arrow.set_UVC(robot_dx, robot_dy)
        
        target_dx = 0.3 * np.cos(target_pose[2])
        target_dy = 0.3 * np.sin(target_pose[2])
        self.target_arrow.set_offsets(np.array([target_pose[0], target_pose[1]]))
        self.target_arrow.set_UVC(target_dx, target_dy)
        
        # Update error text
        self.error_text.set_text(f'Distance error: {ed/100:.2f} m\nYaw error: {eyaw:.2f} deg')
        
        # Auto-adjust plot limits if robot or target goes out of bounds
        xmin, xmax = self.ax.get_xlim()
        ymin, ymax = self.ax.get_ylim()
        
        need_resize = False
        if robot_pose[0] < xmin or robot_pose[0] > xmax or target_pose[0] < xmin or target_pose[0] > xmax:
            xmin = min(xmin, robot_pose[0] - 1, target_pose[0] - 1)
            xmax = max(xmax, robot_pose[0] + 1, target_pose[0] + 1)
            need_resize = True
            
        if robot_pose[1] < ymin or robot_pose[1] > ymax or target_pose[1] < ymin or target_pose[1] > ymax:
            ymin = min(ymin, robot_pose[1] - 1, target_pose[1] - 1)
            ymax = max(ymax, robot_pose[1] + 1, target_pose[1] + 1)
            need_resize = True
        
        if need_resize:
            self.ax.set_xlim(xmin, xmax)
            self.ax.set_ylim(ymin, ymax)
        
        # Redraw plot
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

class FuzzyMembership:
    def __init__(self, labels, centers_or_abc):
        self.labels = labels
        self.is_triangles = isinstance(centers_or_abc[0], (list, tuple, np.ndarray))
        if self.is_triangles:
            self.abc = centers_or_abc
        else:
            self.centers = np.array(centers_or_abc)

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
            memberships[label] = triangleMF(x, a, b, c, LB, UB)
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
            y_vals = [triangleMF(x, a, b, c, LB, UB) for x in x_vals]
            plt.plot(x_vals, y_vals, label=label)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel('Membership Degree')
        plt.grid(True)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5)
        plt.tight_layout()
        plt.show()

class TrackingFIS:
    def __init__(self):
        vel_labels = [
            'TBW', 'VFSBW', 'FSBW', 'BW', 'SLBW', 'VSLBW', 'S',
            'VSLFW', 'SLFW', 'FW', 'FSFW', 'VFSFW', 'TFW'
        ]
        vel_abc = [
            [-400, -350, -300], [-350, -300, -250], [-300, -250, -200], [-250, -200, -150],
            [-200, -150, -100], [-150, -100, -50], [-100, 0, 100], [50, 100, 150],
            [100, 150, 200], [150, 200, 250], [200, 250, 300], [250, 300, 350], [300, 350, 400]
        ]
        self.velocity_mf = FuzzyMembership(vel_labels, vel_abc)

        dist_labels = ['Z', 'VCL', 'CL', 'M', 'F', 'VF', 'IN']
        dist_centers = [0, 50, 100, 150, 200, 250, 300]
        self.distance_mf = FuzzyMembership(dist_labels, dist_centers)

        deg_labels = ['NIN', 'NVL', 'NL', 'NM', 'NS', 'NVS', 'ZE', 'PVS', 'PS', 'PM', 'PL', 'PVL', 'PIN']
        deg_centers = np.linspace(-180, 180, 13)
        self.degree_mf = FuzzyMembership(deg_labels, deg_centers)

        self.vx_rule_table = [
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

        self.vy_rule_table = [
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
        self.vw_out = np.array([0,0])
    
    def update(self, error_vector):
        ed = error_vector[0]
        eyaw = error_vector[1]
        fuzzy_d = self.distance_mf.evaluate(ed)
        fuzzy_t = self.degree_mf.evaluate(eyaw)
        
        numvx, numvy = 0, 0
        denvx, denvy = 0, 0

        for d_label, d_val in fuzzy_d.items():
            for t_label, t_val in fuzzy_t.items():
                d_idx = self.distance_mf.labels.index(d_label)
                t_idx = self.degree_mf.labels.index(t_label)

                vx_out = self.vx_rule_table[t_idx][d_idx]
                vy_out = self.vy_rule_table[t_idx][d_idx]

                rule_strength = min(d_val, t_val)
                
                vx_idx = self.velocity_mf.labels.index(vx_out)
                centroid = self.velocity_mf.abc[vx_idx][1]
                numvx += rule_strength * centroid
                denvx += rule_strength
                
                vy_idx = self.velocity_mf.labels.index(vy_out)
                centroid = self.velocity_mf.abc[vy_idx][1]  
                numvy += rule_strength * centroid
                denvy += rule_strength

        vx = numvx/denvx if denvx > 0 else 0
        vy = numvy/denvy if denvy > 0 else 0
        
        vx /= 1000. 
        vy /= 1000.
        
        max_velocity = 2
        vx = max(-1, min(1, vx / max_velocity))
        vy = max(-1, min(1, vy / max_velocity))
        
        theta = np.arctan2(ey, ex)
        yawd_des = theta - current_pose[2]
        yawd_des = max(-1, min(1, yawd_des))
        
        cmd_wcs = np.array([vx, vy, yawd_des])
        self.vw_out = cmd_wcs2bcs(cmd_wcs, theta)
        return self.vw_out

class ObstacleAvoidanceFIS:    
    def __init__(self):
        self.sensor_being_checked = [1, 2, 3, 4, 5, 6]
        self.weights = [1.5,2.5,3.5]
        self.rule_table_left = np.array([[0, 0], [1, 1]], dtype=int)
        self.rule_table_right = np.array([[0, 1], [0, 1]], dtype=int)
        self.singleton_PWM_outputs = np.array([-5, 5], dtype=float)
        self.max_velocity = 2
        self.vw_out = np.array([0,0])
        self.phi_normal = 5
        self.near_threshold = 0.4
        self.obstacle_nearby = 0
        self.last_obstacle_nearby = 0
        self.lock = 0
        self.last_time = 0

    def ultrasound_membership(self,x):
        near = triangleMF(x, 0.4, 0.4, 0.6, 1, 0)
        far = triangleMF(x, 0.4, 0.6, 0.6, 0, 1)
        y = np.array([near, far], dtype=float)
        return y
    
    def obstacle_gain(self):
        if self.lock:
            return 1
        else:
            return self.obstacle_nearby

    def update(self, distances):
        obj_distance = distances
        if len(obj_distance) < len(self.sensor_being_checked):
            raise ValueError("bluok, salah dimensi")
        
        crisp_out = []
        output_singleton = []
        
        if self.last_obstacle_nearby != self.obstacle_nearby:
            self.lock = True
            self.last_time = time.time()
            self.last_obstacle_nearby = self.obstacle_nearby
        
        if time.time() - self.last_time >= 2 and self.lock == True:
            self.lock = False;
        
        self.obstacle_nearby = 0;
        for _, sensor in enumerate(self.sensor_being_checked):
            distance = obj_distance[sensor]
            distance = max(0.0, min(1.0, distance))
            output = self.ultrasound_membership(distance)
            output_singleton.append([output[0], output[1]])
            if output[0] >= self.near_threshold:
                self.obstacle_nearby = 1
        
        
        
        for idx in range(3):
            left_idx = idx
            right_idx = len(output_singleton) - idx - 1
            
            left_memberships = output_singleton[left_idx]
            right_memberships = output_singleton[right_idx]
            
            num_left = 0
            den_left = 0
            num_right = 0
            den_right = 0
            
            for r in range(2):  # near/far
                for l in range(2):  # near/far
                    tab_idx_left = self.rule_table_left[r][l]
                    tab_idx_right = self.rule_table_right[r][l]
                    
                    fd1andfd2 = float(min(left_memberships[l], right_memberships[r]))
                    
                    num_left += fd1andfd2 * self.singleton_PWM_outputs[tab_idx_left]
                    den_left += fd1andfd2
                    num_right += fd1andfd2 * self.singleton_PWM_outputs[tab_idx_right]
                    den_right += fd1andfd2
            
            crisp_left = num_left / den_left if den_left > 0 else 0
            crisp_right = num_right / den_right if den_right > 0 else 0
            crisp_out.append([crisp_left, crisp_right])
        
        weighted_crisp = [0, 0]
        for i in range(len(crisp_out)):
            weighted_crisp[0] += crisp_out[i][0] * self.weights[i]
            weighted_crisp[1] += crisp_out[i][1] * self.weights[i]
        weighted_crisp[0] /= sum(self.weights)
        weighted_crisp[1] /= sum(self.weights)
        
        weighted_crisp[0] = max(-1, min(1, weighted_crisp[0] / self.max_velocity))
        weighted_crisp[1] = max(-1, min(1, weighted_crisp[1] / self.max_velocity))
        
        # result of this fuzzy is in q space, convert to x_bcs space
        phi_dot_vector = np.array([weighted_crisp[1], weighted_crisp[0]])

        self.vw_out = forwardKinematics(phi_dot_vector)
        return self.vw_out

    def get_output(self):
        return self.vw_out

client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(False)
sim.startSimulation()
sensors_handle = getSensorsHandle(sim)
motors_handle = getMotorsHandle()
phi_normal = 3

obstacle = ObstacleAvoidanceFIS()
tracker = TrackingFIS()
# Add this before your while loop
# position_tracker = PositionTracker()

# Inside your while loop, after calculating ed and eyaw

while True:
    current_time = time.time()
    target_pose = getTargetPose()
    current_pose = getRobotPose()
    ex, ey, eyaw = getErrorPose(target_pose, current_pose)
    obj_distance = getDistances(sim, sensors_handle)
    
    ex *= 100.
    ey *= 100.
    
    ed = np.sqrt(ex**2 + ey**2)
    eyaw = np.degrees(eyaw)
    
    # position_tracker.update(target_pose, current_pose, ed, eyaw)
    if eyaw > 180:
        eyaw -= 360
    elif eyaw < -180:
        eyaw += 360
    
    tracker_vw = tracker.update(np.array([ed, eyaw]))
    obs_vw = obstacle.update(obj_distance)
    
    fused_vw = tracker_vw * (1-obstacle.obstacle_gain()) + obs_vw * obstacle.obstacle_gain()
    
    phi_dot_vector = inverseKinematics(fused_vw)
    
    phi_max = np.max(phi_dot_vector)
    phi_rn = phi_dot_vector[0]
    phi_ln = phi_dot_vector[1]
    
    if phi_max > phi_normal:
        phi_rn = phi_normal * phi_rn / phi_max
        phi_ln = phi_normal * phi_ln / phi_max

    if ed <= 0.05:
        phi_rn = 0
        phi_ln = 0

    wheel_vel = [phi_rn, phi_ln]
    
    setRobotMotion(motors_handle, wheel_vel)