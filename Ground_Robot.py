''' Main ground robot file '''

import sim
import cv2
import numpy as np
import Main_Functions as main_fun
import Quadcopter_Functions as fun
import Ground_Functions as fun_gnd


IMG_WIDTH = 512
IMG_HEIGHT = 512
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
        clientID, 'ground_vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Floor proximity ir sensor
    res, ir_sensor = sim.simxGetObjectHandle(
        clientID, 'ground_IR_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Proximity Sensor')

    # Ground robot body
    res, body = sim.simxGetObjectHandle(
        clientID, 'Shape', sim.simx_opmode_oneshot_wait)
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
        # Get image from Camera
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_buffer)

        # unique_values, delta = main_fun.currentVisionState(
        #     'FINDING_TEDDY', res, resolution, image_gnd)
        tshirt_x = IMG_WIDTH/2

        if res == sim.simx_return_ok:
            original = np.array(image, dtype=np.uint8)
            original.resize([resolution[0], resolution[1], 3])
            original = cv2.flip(original, 0)
            original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

            # Find mask for Mr York's tshirt
            tshirt_mask = fun_gnd.findBearMask(original)

            values = list(tshirt_mask)
            unique_values = np.unique(values)

            # Finding centre of mass of Mr York's tshirt
            tshirt_image, tshirt_x, tshirt_y = fun.detectCenterOfMass(
                tshirt_mask, True)
            cv2.imshow("Center_of_Tshirt", tshirt_image)

        elif res == sim.simx_return_novalue_flag:
            # Camera has not started or is not returning images
            print("Wait, there's no image yet")
        else:
            # Something else has happened
            print("Unexpected error returned", res)

        # Calculated offset
        delta = fun_gnd.controllerMove(tshirt_x)

        # Look up for Mr York and move towards him
        if (len(unique_values) == 1):
            if (unique_values[0] == 0):
                fun_gnd.groundMovement(
                    'TURN_RIGHT', clientID, leftMotor, rightMotor, delta)
            if (unique_values[0] == 255):
                fun_gnd.groundMovement(
                    'STOP', clientID, leftMotor, rightMotor, delta)
        else:
            fun_gnd.groundMovement('FORWARD', clientID,
                                   leftMotor, rightMotor, delta)

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break

        # ----------- IDEA THAT REQUIRES DEVELOPMENT -----------
        # Here the idea is to continue moving forward towards Mr York
        # until the proximity sensor detects something.
        res, isDetecting, point_detected, num, vector = sim.simxReadProximitySensor(
            clientID, ir_sensor, sim.simx_opmode_oneshot)
        if isDetecting:
            print("Mr York detected")
            print("Point coordinates: ", point_detected)

        '''
        res, position = sim.simxGetObjectPosition(
            clientID, body_gnd, floor, sim.simx_opmode_oneshot)
        if res == sim.simx_return_ok:
            print("X: ", round(position[0], 0))
            print("Y: ", round(position[1], 0))
            print("Z: ", round(position[2], 0))'''

else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
