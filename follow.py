from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import matplotlib.pyplot as plt

def getSensorsHandle():
    sensorsHandle = np.array([])
    for i in range(16):
        sensorHandle = sim.getObject("/PioneerP3DX/ultrasonicSensor[" + str(i) + "]")
        sensorsHandle = np.append(sensorsHandle, sensorHandle)
    _, _, _, _, _ = sim.handleProximitySensor(sim.handle_all)
    return sensorsHandle

def getDistances(sensorsHandle):
    Distances = np.array([])
    for i in range(16):
        detectionState, _, detectedPoint, _, _ = sim.readProximitySensor(
            sensorsHandle[i]
        )
        distanceValue = detectedPoint[2]
        if detectionState == False:
            distanceValue = 2.0
        Distances = np.append(Distances, distanceValue)
    return Distances

def getMotorsHandle():
    motorRightHandle = sim.getObject("/PioneerP3DX/rightMotor")
    motorLeftHandle = sim.getObject("/PioneerP3DX/leftMotor")
    return (motorRightHandle, motorLeftHandle)

def setRobotMotion(motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])
    return

print("Program Started")

client = RemoteAPIClient()
sim = client.require("sim")

sim.setStepping(False)
sim.startSimulation()

sensors_handle = getSensorsHandle()
motors_handle = getMotorsHandle()

d_ref = 0.3
theta_ref = 0.0  
K1 = -1
K2 = -1  
R = 0.0975 # radius of the wheel
L = 0.381 # distance between the wheels
phi_norm = 2
stop_after = 30

time_start = sim.getSimulationTime()
time_data = []
distance_data = []
orientation_data = []

while True:
    t_now = sim.getSimulationTime() - time_start
    obj_distances = getDistances(sensors_handle)

    d = min(obj_distances[3], obj_distances[4])  # S4 and S5, index starts from 1 duh
    theta = obj_distances[5] - obj_distances[2]  # S6 and S3, index starts from 1 duh
    
    v = K1 * (d_ref - d)
    omega = K2 * (theta_ref - theta)

    ik_mat = np.array([[R/2, R/2], [R/(2*L), -R/(2*L)]])
    velocity_vector = np.array([v, omega])
    wheel_velocities = np.linalg.inv(ik_mat).dot(velocity_vector)

    phi_R = wheel_velocities[0]
    phi_L = wheel_velocities[1]
    phi_max = max(phi_R, phi_L)

    if phi_max > phi_norm:
        phi_RN = (phi_norm / phi_max) * phi_R
        phi_LN = (phi_norm / phi_max) * phi_L
    else:
        phi_RN = phi_R
        phi_LN = phi_L

    wheels_velo = [phi_RN, phi_LN]
    # wheels_velo = [0, 0]
    
    setRobotMotion(motors_handle, wheels_velo)
    
    time_data.append(t_now)
    distance_data.append(d)
    orientation_data.append(theta)
    
    # Exit Condition
    if sim.getSimulationTime() - time_start > stop_after:
        break

sim.stopSimulation()
print("\nProgram Ended\n")

plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.plot(time_data, distance_data)
plt.title("Distance vs. Time")
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")

plt.subplot(1, 2, 2)
plt.plot(time_data, orientation_data)
plt.title("Orientation vs. Time")
plt.xlabel("Time (s)")
plt.ylabel("Orientation (rad)")

plt.tight_layout()
plt.show()