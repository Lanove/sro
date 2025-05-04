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

def triangular_membership(x, a, b, c, d, e):
    if x < a:
        return d
    elif a <= x <= b:
        return (x - a) / (b - a)
    elif b <= x <= c:
        return (c - x) / (c - b)
    else:  # x > c
        return e

def ultrasound_membership(distance):
  near = triangular_membership(distance, 0.1, 0.1, 0.9, 1, 0)
  far = triangular_membership(distance, 0.1, 0.9, 0.9, 0 ,1)
  
  return near, far

dis_eval = np.linspace(0, 1, 100)

near_vals = []
far_vals = []

for dis in dis_eval:
    near, far = ultrasound_membership(dis)
    near_vals.append(near)
    far_vals.append(far)

plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))
plt.title('Distance Membership Functions')
plt.xlabel('Distance (m)')
plt.ylabel('Membership Value')
plt.grid(True)
plt.xlim([0, 1])
plt.ylim([0, 1.1])

ax.plot(dis_eval, near_vals, 'b-', label='Near')
ax.plot(dis_eval, far_vals, 'g-', label='Far')
ax.fill_between(dis_eval, near_vals, alpha=0.2, color='blue')
ax.fill_between(dis_eval, far_vals, alpha=0.2, color='green')
plt.legend()

dot0_near, = ax.plot([], [], 'ro', markersize=8, label='S0')
dot0_far, = ax.plot([], [], 'ro', markersize=8, fillstyle='none', label='S0')

dot2_near, = ax.plot([], [], 'go', markersize=8, label='S2')
dot2_far, = ax.plot([], [], 'go', markersize=8, fillstyle='none', label='S2')

dot5_near, = ax.plot([], [], 'bo', markersize=8, label='S5')
dot5_far, = ax.plot([], [], 'bo', markersize=8, fillstyle='none', label='S5')

dot7_near, = ax.plot([], [], 'mo', markersize=8, label='S7')
dot7_far, = ax.plot([], [], 'mo', markersize=8, fillstyle='none', label='S7')
ax.legend(loc='upper right')

text0 = ax.text(0.05, 0.95, '', transform=ax.transAxes)
text2 = ax.text(0.05, 0.90, '', transform=ax.transAxes)
text5 = ax.text(0.05, 0.85, '', transform=ax.transAxes)
text7 = ax.text(0.05, 0.80, '', transform=ax.transAxes)
text_wheels = ax.text(0.05, 0.75, f'', transform=ax.transAxes)

plt.draw()
plt.pause(0.01)

print("Program Started")

client = RemoteAPIClient()
sim = client.require("sim")

sim.setStepping(False)
sim.startSimulation()

sensors_handle = getSensorsHandle()
motors_handle = getMotorsHandle()

stop_after = 30000

time_start = sim.getSimulationTime()

while True:
    t_now = sim.getSimulationTime() - time_start
    obj_distances = getDistances(sensors_handle)

    d0, d2, d5, d7 = obj_distances[0], obj_distances[2], obj_distances[5], obj_distances[7]
    
    d0_clip = min(d0, 1.0)
    d2_clip = min(d2, 1.0)
    d5_clip = min(d5, 1.0)
    d7_clip = min(d7, 1.0)

    s0_near, s0_far = ultrasound_membership(d0_clip)
    s2_near, s2_far = ultrasound_membership(d2_clip)
    s5_near, s5_far = ultrasound_membership(d5_clip)
    s7_near, s7_far = ultrasound_membership(d7_clip)
    
    dot0_near.set_data([d0_clip], [s0_near])
    dot0_far.set_data([d0_clip], [s0_far])

    dot2_near.set_data([d2_clip], [s2_near])
    dot2_far.set_data([d2_clip], [s2_far])

    dot5_near.set_data([d5_clip], [s5_near])
    dot5_far.set_data([d5_clip], [s5_far])

    dot7_near.set_data([d7_clip], [s7_near])
    dot7_far.set_data([d7_clip], [s7_far])
    
    text0.set_text(f'S0: {d0:.2f}m (N: {s0_near:.2f}, F: {s0_far:.2f})')
    text2.set_text(f'S2: {d2:.2f}m (N: {s2_near:.2f}, F: {s2_far:.2f})')
    text5.set_text(f'S5: {d5:.2f}m (N: {s5_near:.2f}, F: {s5_far:.2f})')
    text7.set_text(f'S7: {d7:.2f}m (N: {s7_near:.2f}, F: {s7_far:.2f})')
    
    fig.canvas.draw_idle()
    plt.pause(0.01)
    
    wheels_velo = [-0.5, 0.5]
    
    setRobotMotion(motors_handle, wheels_velo)
    
    # Exit Condition
    if sim.getSimulationTime() - time_start > stop_after:
        break
sim.stopSimulation()
print("\nProgram Ended\n")