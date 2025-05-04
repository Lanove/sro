from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import time
import numpy as np

R = 0.0975 # radius of the wheel
L = 0.381 # distance between the wheels
DEBOUNCE_TIME = 0.1  # 100ms debounce time
last_key_press = {
    "f": 0,
    "b": 0,
    "l": 0,
    "r": 0,
    "q": 0
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
lin_vel_lim = 0.25
ang_vel_lim = 0.25

while True:
    if keyboard.is_pressed("esc"):
        break
    
    current_time = time.time()
    if keyboard.is_pressed("f") and (current_time - last_key_press["f"]) > DEBOUNCE_TIME:
        cmd_vel[0] = lin_vel_lim
        last_key_press["f"] = current_time
    if keyboard.is_pressed("b") and (current_time - last_key_press["b"]) > DEBOUNCE_TIME:
        cmd_vel[0] = -lin_vel_lim
        last_key_press["b"] = current_time
    if keyboard.is_pressed("l") and (current_time - last_key_press["l"]) > DEBOUNCE_TIME:
        cmd_vel[1] = ang_vel_lim
        last_key_press["l"] = current_time
    if keyboard.is_pressed("r") and (current_time - last_key_press["r"]) > DEBOUNCE_TIME:
        cmd_vel[1] = -ang_vel_lim
        last_key_press["r"] = current_time
    if keyboard.is_pressed("q") and (current_time - last_key_press["q"]) > DEBOUNCE_TIME:
        cmd_vel = [0.0, 0.0]
        last_key_press["q"] = current_time
    
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
