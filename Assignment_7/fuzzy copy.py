import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def triangle_membership(x, a, b, c, FD0, FD2):
    if x < a:
        FD = FD0
    elif x >= a and x < b:
        FD = (x - a) / (b - a)
    elif x >= b and x < c:
        FD = (c - x) / (c - b)
    elif x >= c:
        FD = FD2
    return FD

def ultrasound_membership(x):
    near = triangle_membership(x, 0.2, 0.2, 0.3, 1, 0)
    far = triangle_membership(x, 0.2, 0.3, 0.3, 0, 1)
    y = np.array([[near], [far]], dtype=float)
    return y

def getSensorsHandle(sim):
    sensorsHandle = []
    for i in range(16):
        sensorHandle = sim.getObject('/PioneerP3DX/ultrasonicSensor[' + str(i) + ']')
        sensorsHandle.append(sensorHandle)
    _, _, _, _, _ = sim.handleProximitySensor(sim.handle_all)
    return sensorsHandle

def getDistances(sim, sensorsHandle):
    Distances = []
    for i in range(16):
        detectionState, _, detectedPoint, _, _ = sim.readProximitySensor(sensorsHandle[i])
        distanceValue = detectedPoint[2]
        if detectionState == False:
            distanceValue = 1.0
        Distances.append(distanceValue)
    return Distances

def getMotorsHandle(sim):
    motorRightHandle = sim.getObject('/PioneerP3DX/rightMotor')
    motorLeftHandle = sim.getObject('/PioneerP3DX/leftMotor')
    return motorLeftHandle, motorRightHandle

def setRobotMotion(sim, motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])

client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(False)
sim.startSimulation()

sensors_handle = getSensorsHandle(sim)
motors_handle = getMotorsHandle(sim)
wheels_velo = [0.0, 0.0]
singleton_PWM_outputs = np.array([[-5], [5]], dtype=float)
weights = [0.5, 1.0, 1.5]
sensor_being_checked = [1, 2, 3, 4, 5, 6]
rule_table_left = np.array([[0, 0], [1, 1]], dtype=int)
rule_table_right = np.array([[0, 1], [0, 1]], dtype=int)
lt = ["near", "far"]
stop_after = 30000
time_start = sim.getSimulationTime()

while True:
    t_now = sim.getSimulationTime() - time_start
    obj_distance = getDistances(sim, sensors_handle)
    d0, d2, d5, d7 = obj_distance[0], obj_distance[2], obj_distance[5], obj_distance[7]
    d0_clip = min(d0, 1.0)
    d2_clip = min(d2, 1.0)
    d5_clip = min(d5, 1.0)
    d7_clip = min(d7, 1.0)
    
    output_singleton = []
    crisp_out = []
    
    for sensor_idx, sensor in enumerate(sensor_being_checked):
        distance = obj_distance[sensor]
        distance = max(0.0, min(1.0, distance))
        output = ultrasound_membership(distance)
        output_singleton.append([output[0][0], output[1][0]])
    
    defuzz_table = []
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
                tab_idx_left = rule_table_left[r][l]
                tab_idx_right = rule_table_right[r][l]
                
                fd1andfd2 = float(min(left_memberships[l], right_memberships[r]))
                
                num_left += fd1andfd2 * singleton_PWM_outputs[tab_idx_left][0]
                den_left += fd1andfd2
                num_right += fd1andfd2 * singleton_PWM_outputs[tab_idx_right][0]
                den_right += fd1andfd2
        
        crisp_left = num_left / den_left if den_left > 0 else 0
        crisp_right = num_right / den_right if den_right > 0 else 0
        crisp_out.append([crisp_left, crisp_right])
    
    weighted_crisp = [0, 0]
    for i in range(len(crisp_out)):
        weighted_crisp[0] += crisp_out[i][0] * weights[i]
        weighted_crisp[1] += crisp_out[i][1] * weights[i]
    weighted_crisp[0] /= sum(weights)
    weighted_crisp[1] /= sum(weights)
    
    max_velocity = 2
    weighted_crisp[0] = max(-1, min(1, weighted_crisp[0] / max_velocity))
    weighted_crisp[1] = max(-1, min(1, weighted_crisp[1] / max_velocity))
    
    wheels_velo = weighted_crisp[0], weighted_crisp[1]
    text_wheels.set_text(f'Wheels: L={wheels_velo[0]:.2f}, R={wheels_velo[1]:.2f}')
    
    setRobotMotion(sim, motors_handle, wheels_velo)
    fig.canvas.draw_idle()
    plt.pause(0.01)
    
    if sim.getSimulationTime() - time_start > stop_after:
        break

sim.stopSimulation()
print("\nProgram Ended\n")