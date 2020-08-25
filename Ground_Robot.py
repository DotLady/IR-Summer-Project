''' Main ground robot file '''

import sim
import cv2
import numpy as np
import Main_Functions as main_fun
import Quadcopter_Functions as fun
import Ground_Functions as fun_gnd


IMG_WIDTH = 512
IMG_HEIGHT = 512
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

    isRescueDone = False
    isReadyToSearch = False
    unique_values = []

    while (sim.simxGetConnectionId(clientID) != -1):
        # Get image from Camera
        res_camera, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_buffer)
        res_left_joint, left_joint_pos = sim.simxGetJointPosition(
            clientID, left_joint, sim.simx_opmode_oneshot)
        res_right_joint, right_joint_pos = sim.simxGetJointPosition(
            clientID, right_joint, sim.simx_opmode_oneshot)

        if (not isRescueDone):
            if (left_joint_pos < (np.pi-0.15)) and (right_joint_pos < 0.15):
                if (res_left_joint == sim.simx_return_ok) and (res_right_joint == sim.simx_return_ok):
                    sim.simxSetJointTargetVelocity(
                        clientID, left_joint, 0.2, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(
                        clientID, right_joint, -0.2, sim.simx_opmode_oneshot)
            else:
                sim.simxSetJointTargetVelocity(
                    clientID, left_joint, 0.0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID, right_joint, 0.0, sim.simx_opmode_oneshot)
                isReadyToSearch = True
        else:
            # if (left_joint_pos > 0):
            if (res_left_joint == sim.simx_return_ok) and (res_right_joint == sim.simx_return_ok):
                sim.simxSetJointTargetVelocity(
                    clientID, left_joint, -0.2, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID, right_joint, 0.2, sim.simx_opmode_oneshot)
            # else:
            #     sim.simxSetJointTargetVelocity(
            #         clientID, left_joint, 0.0, sim.simx_opmode_oneshot)
            #     sim.simxSetJointTargetVelocity(
            #         clientID, right_joint, 0.0, sim.simx_opmode_oneshot)

        # unique_values, delta = main_fun.currentVisionState(
        #     'FINDING_TEDDY', res, resolution, image_gnd)
        tshirt_x = IMG_WIDTH/2.0

        if (res_camera == sim.simx_return_ok) and isReadyToSearch:
            original = np.array(image, dtype=np.uint8)
            original.resize([resolution[0], resolution[1], 3])
            original = cv2.flip(original, 0)
            original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

            # Find mask for Mr York's tshirt
            tshirt_mask = fun_gnd.findBearMask(original)

            values = list(tshirt_mask)
            unique_values = np.unique(values)

            # Finding centre of mass of Mr York's tshirt
            tshirt_image, tshirt_x, tshirt_y = fun_gnd.detectCenterOfMass(
                tshirt_mask, True)
            # cv2.imshow("Center_of_Tshirt", tshirt_image)

        # elif res == sim.simx_return_novalue_flag:
        #     # Camera has not started or is not returning images
        #     print("Wait, there's no image yet")
        # else:
        #     # Something else has happened
        #     print("Unexpected error returned", res)

        if isReadyToSearch:
            # Calculated offset
            delta = fun_gnd.controllerMove(tshirt_x)

            # Look up for Mr York and move towards him
            if (len(unique_values) == 1):
                if (unique_values[0] == 0):
                    fun_gnd.groundMovement(
                        'TURN_RIGHT', clientID, leftMotor, rightMotor, SPEED, delta)
                elif (unique_values[0] == 255):
                    fun_gnd.groundMovement(
                        'STOP', clientID, leftMotor, rightMotor, SPEED, delta)
                    isRescueDone = True
                    isReadyToSearch = False

            else:
                fun_gnd.groundMovement('FORWARD', clientID,
                                       leftMotor, rightMotor, SPEED, delta)
                res, isDetecting, point_detected, num, vector = sim.simxReadProximitySensor(
                    clientID, prox_sensor, sim.simx_opmode_oneshot)
                if isDetecting:
                    # print("Mr York detected")
                    fun_gnd.groundMovement(
                        'STOP', clientID, leftMotor, rightMotor, SPEED, delta)
                    isRescueDone = True
                    isReadyToSearch = False

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break


else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
