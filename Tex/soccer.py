from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import time
import numpy as np

R = 0.0975 # radius of the wheel
L = 0.381 # distance between the wheels
DEBOUNCE_TIME = 0.1  # 100ms debounce time
last_key_press = {
    "up": 0,
    "down": 0,
    "left": 0,
    "right": 0,
    "r": 0
}

def inverseKinematics(v, w):
    inverse_matrix = np.linalg.inv(np.array([[R/2, R/2], [R/(2*L), -R/(2*L)]]))
    return np.dot(inverse_matrix, [v, w])

def getMotorsHandle():
    motorRightHandle = sim.getObject("/Robot_Pemain/rightMotor")
    motorLeftHandle = sim.getObject("/Robot_Pemain/leftMotor")
    return (motorRightHandle, motorLeftHandle)

def setRobotMotion(motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])
    return

client = RemoteAPIClient()
sim = client.require("sim")

sim.setStepping(False)
sim.startSimulation()

motors_handle = getMotorsHandle()

cmd_vel = [0.0, 0.0]
prev_cmd_vel = [0.0, 0.0]
wheel_vel = [0.0, 0.0]
lin_vel_lim = 1
ang_vel_lim = 0.5

while True:
    if keyboard.is_pressed("esc"):
        break
    
    current_time = time.time()
    if keyboard.is_pressed("up") and (current_time - last_key_press["up"]) > DEBOUNCE_TIME:
        cmd_vel[0] += 0.1
        last_key_press["w"] = current_time
    if keyboard.is_pressed("down") and (current_time - last_key_press["down"]) > DEBOUNCE_TIME:
        cmd_vel[0] -= 0.1
        last_key_press["s"] = current_time
    if keyboard.is_pressed("left") and (current_time - last_key_press["left"]) > DEBOUNCE_TIME:
        cmd_vel[1] += 0.1
        last_key_press["a"] = current_time
    if keyboard.is_pressed("right") and (current_time - last_key_press["right"]) > DEBOUNCE_TIME:
        cmd_vel[1] -= 0.1
        last_key_press["d"] = current_time
    if keyboard.is_pressed("r") and (current_time - last_key_press["r"]) > DEBOUNCE_TIME:
        cmd_vel = [0.0, 0.0]
        last_key_press["r"] = current_time
    
    if cmd_vel[0] > lin_vel_lim:
        cmd_vel[0] = lin_vel_lim
    if cmd_vel[0] < -lin_vel_lim:
        cmd_vel[0] = -lin_vel_lim
    if cmd_vel[1] > ang_vel_lim:
        cmd_vel[1] = ang_vel_lim
    if cmd_vel[1] < -ang_vel_lim:
        cmd_vel[1] = -ang_vel_lim 
    
    wheel_vel = inverseKinematics(cmd_vel[0], cmd_vel[1])
    
    if cmd_vel != prev_cmd_vel:
        print(f"cmd_vel: {cmd_vel}, wheel_vel: {wheel_vel}")
    
    setRobotMotion(motors_handle, wheel_vel)
    prev_cmd_vel = cmd_vel.copy()


sim.stopSimulation()
print("\nProgram Ended\n")
