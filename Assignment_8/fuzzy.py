from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# import keyboard
import time
import numpy as np

R = 0.0975 # radius of the wheel
L = 0.381 # distance between the wheels

def inverseKinematics(v, w):
    inverse_matrix = np.linalg.inv(np.array([[R/2, R/2], [R/(2*L), -R/(2*L)]]))
    return np.dot(inverse_matrix, [v, w])

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

print("HRE")
client = RemoteAPIClient()
sim = client.require("sim")
print("DASKDOAS")
sim.setStepping(False)
sim.startSimulation()

error_tolerance = 0.05
gain = [1,1,1]
phi_normal = 1

motors_handle = getMotorsHandle()

while True:
    current_time = time.time()
    target_pose = getTargetPose()
    current_pose = getRobotPose()

    wheel_vel = [0, 0]
    
    setRobotMotion(motors_handle, wheel_vel)