''' Main ground robot file '''

import sim
import cv2
import numpy as np
import Main_Functions as main_fun
import Quadcopter_Functions as fun
import Ground_Functions as fun_gnd


IMG_WIDTH = 512
IMG_HEIGHT = 512
onenunitdistance = 6.283185307179586
groundrobot_forwardspeed = 5.0
pathlist = [[-5.0, -7.0, 0.0],
            [-5.0, -5.0, 90.0],
            [-4.0, -5.0, 0.0]]
SPEED = 5.0
delta = 0.0


# -------------------------------------- START PROGRAM --------------------------------------
# Start Program and just in case, close all opened connections
print('Program started')
sim.simxFinish(-1)

# Connect to simulator running on localhost
clientID = sim.simxStart('127.0.0.1', 19998, True, True, 5000, 5)

# Connect to the simulation
if clientID != -1:
    print('Connected to remote API server')

    # Get handles to simulation objects
    print('Obtaining handles of simulation objects...')

    # Ground robot's perspective vision sensor
    res, camera = sim.simxGetObjectHandle(
        clientID, 'Ground_vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Floor proximity ir sensor
    res, prox_sensor = sim.simxGetObjectHandle(
        clientID, 'proximity_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Proximity Sensor')

    # Ground robot body
    res, body = sim.simxGetObjectHandle(
        clientID, 'Shape', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    # Ground robot arm joints
    res, left_joint = sim.simxGetObjectHandle(
        clientID, 'LeftJoint', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    res, right_joint = sim.simxGetObjectHandle(
        clientID, 'RightJoint', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    res, elevation_motor = sim.simxGetObjectHandle(
        clientID, 'Elevation_motor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    # Wheel drive motors
    res, leftMotor = sim.simxGetObjectHandle(
        clientID, 'leftMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to leftMotor')
    res, rightMotor = sim.simxGetObjectHandle(
        clientID, 'rightMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to rightMotor')

    # Wheels
    res, leftWheel = sim.simxGetObjectHandle(
        clientID, 'leftWheel', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to leftWheel')
    res, rightWheel = sim.simxGetObjectHandle(
        clientID, 'rightWheel', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to rightWheel')

    # Floor
    res, floor = sim.simxGetObjectHandle(
        clientID, 'ResizableFloor_5_25', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Floor')


# -------------------------------------- Main Control Loop --------------------------------------

    # Start main control loop
    print('Starting control loop')

    res, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera, 0, sim.simx_opmode_streaming)

    while (sim.simxGetConnectionId(clientID) != -1):
        res, elevation_motor_position = sim.simxGetJointPosition(
            clientID, elevation_motor, sim.simx_opmode_oneshot)

        if (res == sim.simx_return_ok):
            if (elevation_motor_position < 0.05):
                sim.simxSetJointTargetVelocity(
                    clientID, elevation_motor, 0.1, sim.simx_opmode_oneshot)
            elif (elevation_motor_position > 0.0):
                sim.simxSetJointTargetVelocity(
                    clientID, elevation_motor, -0.1, sim.simx_opmode_oneshot)
        # else:
        #     if (res == sim.simx_return_ok):
        #         sim.simxSetJointTargetVelocity(
        #             clientID, elevation_motor, -0.1, sim.simx_opmode_oneshot)

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break


else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
