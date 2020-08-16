import sim
import cv2
import numpy as np
import Main_Functions as fun
import Main_States as states
import tcod

IMG_WIDTH = 512
IMG_HEIGHT = 512

grid_matrix = np.zeros((512, 512))

# -------------------------------------- START PROGRAM --------------------------------------
# Start Program and just in case, close all opened connections
print('Program started')
sim.simxFinish(-1)

# Connect to simulator running on localhost
# V-REP runs on port 19997, a script opens the API on port 19999
clientID_drone = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
clientID_gnd = sim.simxStart('127.0.0.1', 19998, True, True, 5000, 5)

# Connect to the simulation
if (clientID_drone != -1) and (clientID_gnd != -1):
    print('Connected to remote API server')

    # Get handles to simulation objects
    print('Obtaining handles of simulation objects...')

    camera_drone, body_drone, camera_gnd, ir_sensor, body_gnd, leftMotor, rightMotor, floor = fun.handleObjects(
        clientID_drone, clientID_gnd)

    # Start main control loop
    print('Starting control loop')

    res_drone, resolution_drone, image_drone = sim.simxGetVisionSensorImage(
        clientID_drone, camera_drone, 0, sim.simx_opmode_streaming)
    res_gnd, resolution_gnd, image_gnd = sim.simxGetVisionSensorImage(
        clientID_gnd, camera_gnd, 0, sim.simx_opmode_streaming)

    isMapReady = False
    isLookingForBear = False

    while (sim.simxGetConnectionId(clientID_drone) != -1) and (sim.simxGetConnectionId(clientID_gnd) != -1):

        delta = 0.0
        commands = []

        if not isMapReady:
            res_drone, resolution_drone, image_drone = sim.simxGetVisionSensorImage(
                clientID_drone, camera_drone, 0, sim.simx_opmode_buffer)

            if res_drone == sim.simx_return_ok:
                original = np.array(image_drone, dtype=np.uint8)
                original.resize([resolution_drone[0], resolution_drone[1], 3])
                original = cv2.flip(original, 0)
                original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

                path_image, commands, isMapReady = states.GETTING_MAP(original)

                cv2.imshow("Path", path_image)

                if isMapReady:
                    print("\nPath coordinates are ready!")
                    isLookingForBear = False

            elif res_drone == sim.simx_return_novalue_flag:
                print("Wait, there's no image yet...")
            else:
                print("Unexpected error returned", res_drone)

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break

        # Searching and moving towards Mr York

        if isLookingForBear:
            res_gnd, resolution_gnd, image_gnd = sim.simxGetVisionSensorImage(
                clientID_gnd, camera_gnd, 0, sim.simx_opmode_buffer)

            if res == sim.simx_return_ok:
                original = np.array(image_gnd, dtype=np.uint8)
                original.resize([resolution_gnd[0], resolution_gnd[1], 3])
                original = cv2.flip(original, 0)
                original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

                color_values, delta = states.SEARCHING_BEAR(
                    original, commands, clientID_gnd, body_gnd, floor, leftMotor, rightMotor)

            elif res == sim.simx_return_novalue_flag:
                print("Wait, there's no image yet")
            else:
                print("Unexpected error returned", res)

            # Look up for Mr York and move towards him
            if (len(color_values) == 1):
                if (color_values[0] == 0):
                    fun.groundMovement(
                        'TURN_RIGHT', clientID, leftMotor, rightMotor, delta)
                if (color_values[0] == 255):
                    fun.groundMovement(
                        'STOP', clientID, leftMotor, rightMotor, delta)
            else:
                fun.groundMovement('FORWARD', clientID,
                                   leftMotor, rightMotor, delta)

            fun.groundMovement('STOP', clientID_gnd,
                               leftMotor, rightMotor, delta)

        # # ----------- IN CASE WE USE IR SENSOR -----------
        # # Here the idea is to continue moving forward towards Mr York
        # # until the proximity sensor detects something.
        # # res, isDetecting, point_detected, num, vector = sim.simxReadProximitySensor(
        # #     clientID, ir_sensor, sim.simx_opmode_oneshot)
        # # if isDetecting:
        # #     print("Mr York detected")
        # #     print("Point coordinates: ", point_detected)

else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID_drone)
sim.simxFinish(clientID_gnd)
cv2.destroyAllWindows()
print('Simulation ended')
