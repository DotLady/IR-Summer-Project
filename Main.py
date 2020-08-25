import sim
import cv2
import numpy as np
import math
import Main_Functions as fun
import Main_States as states
import tcod
import time

IMG_WIDTH = 512
IMG_HEIGHT = 512

drone_viewposition = [0, 0, 8]
repeatseed = 0
grid_matrix = np.zeros((512, 512))

# Yaowen
delta = 0.0
lastLeftWheelPosition = 0.0
lastRightWheelPosition = 0.0
leftWheelOdom = 0.0
onenunitdistance = 6.283185307179586
groundrobot_forwardspeed = 5.0

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

    camera_drone, drone_base_handle, drone_target_handle, floor = fun.handleDroneObjects(
        clientID_drone)
    camera_gnd, prox_sensor, body_gnd, left_motor, right_motor, leftWheel, rightWheel, left_joint, right_joint = fun.handleGroundObjects(
        clientID_gnd)

    # Start main control loop
    print('Starting...')

    res_drone, resolution_drone, image_drone = sim.simxGetVisionSensorImage(
        clientID_drone, camera_drone, 0, sim.simx_opmode_streaming)
    res_gnd, resolution_gnd, image_gnd = sim.simxGetVisionSensorImage(
        clientID_gnd, camera_gnd, 0, sim.simx_opmode_streaming)

    # Yaowen
    sim.simxSetJointTargetVelocity(
        clientID_gnd, left_motor, 5.0, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID_gnd, right_motor, 5.0, sim.simx_opmode_oneshot)

    # Ground robot move forward and get the diameter
    # sim.simxSetJointTargetVelocity(
    #     clientID_gnd, left_motor, groundrobot_forwardspeed, sim.simx_opmode_oneshot)
    # sim.simxSetJointTargetVelocity(
    #     clientID_gnd, right_motor, groundrobot_forwardspeed, sim.simx_opmode_oneshot)
    leftWheelDiameter = \
        sim.simxGetObjectFloatParameter(clientID_gnd, leftWheel, 18, sim.simx_opmode_oneshot)[1] \
        - sim.simxGetObjectFloatParameter(clientID_gnd,
                                          leftWheel, 15, sim.simx_opmode_oneshot)[1]

    isDroneCentered = False
    isMapReady = False
    isPreparedToGo = False
    isMovingToManta = False
    isLookingForBear = False
    isRescueDone = False
    isReadyToSearch = False

    delta = 0.0
    commands = []
    color_values = []

    while (sim.simxGetConnectionId(clientID_drone) != -1) and (sim.simxGetConnectionId(clientID_gnd) != -1):

        # -------------------------------------- DRONE MOVES TO THE CENTRE OF THE SCENE --------------------------------------

        # Moving drone to the center of the environment
        if (not isDroneCentered):
            drone_viewposition, repeatseed, isDroneCentered = fun.droneInitialMovement(
                clientID_drone, drone_base_handle, drone_target_handle, floor, drone_viewposition, repeatseed)

        # If we don't have a path
        if (not isMapReady) and isDroneCentered:
            res_drone, resolution_drone, image_drone = sim.simxGetVisionSensorImage(
                clientID_drone, camera_drone, 0, sim.simx_opmode_buffer)

            if res_drone == sim.simx_return_ok:
                time.sleep(40)
                original = np.array(image_drone, dtype=np.uint8)
                original.resize([resolution_drone[0], resolution_drone[1], 3])
                original = cv2.flip(original, 0)
                original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

                # Image processing with the drone's camera. Getting a path with A* algorithm
                commands, isMapReady = states.GETTING_MAP(original)

                if isMapReady:
                    print("\nPath is ready!")
                    print(commands)
                    isMovingToManta = True
                    isLookingForBear = False

        # -------------------------------------- GROUND ROBOT MOVES TOWARDS RED MANTA --------------------------------------

        # if isMapReady:
        #     isPreparedToGo = fun.openArms(
        #         clientID_gnd, left_joint, right_joint, isPreparedToGo)

        if isMapReady and isMovingToManta:  # and isPreparedToGo:
            # isMovingToManta, isLookingForBear = states.FOLLOWING_PATH(
            #     state, commands, clientID_gnd, body_gnd, floor, left_motor, right_motor)

            # Function of wheel odometery
            leftWheelPosition = sim.simxGetJointPosition(
                clientID_gnd, left_motor, sim.simx_opmode_oneshot)[1]
            dTheta = leftWheelPosition - lastLeftWheelPosition
            if dTheta >= np.pi:
                dTheta -= 2*np.pi
            elif dTheta < np.pi:
                dTheta += 2*np.pi
            leftWheelOdom += dTheta * leftWheelDiameter / 2
            lastLeftWheelPosition = leftWheelPosition

            # Function that used to test the wheelOdom reading for moving 1 unit
            # if -6.0<=groundrobotposition[1][0] <=-5.9:
            #     sim.simxSetJointTargetVelocity(
            #         clientID, left_motor, 0.0, sim.simx_opmode_oneshot)
            #     sim.simxSetJointTargetVelocity(
            #         clientID, right_motor, 0.0, sim.simx_opmode_oneshot)
            #
            # Update the position
            groundrobotposition = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
            groundrobotangle_Euler = sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2]
            groundrobotangle = fun.changeangletoeurrle(
                groundrobotangle_Euler)
            print(groundrobotangle)
            # If get a path list
            if (len(commands) >= 1):
                # Check if the angle of ground robot is same as the list, if true check position else robot rotating
                if (fun.checkangle(groundrobotangle, commands[0][2]) == True):
                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, left_motor, 0.0, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, right_motor, 0.0, sim.simx_opmode_oneshot)
                    groundrobotposition = sim.simxGetObjectPosition(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
                    groundrobotangle_Euler = sim.simxGetObjectOrientation(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2]
                    groundrobotangle = fun.changeangletoeurrle(
                        groundrobotangle_Euler)
                    # Function to check position, [x,y] x y are bool values, 1 means two positions are same, 0 means different
                    # will return [1,1] when arrive the position.
                    if (fun.checkposition(groundrobotposition[1][0], groundrobotposition[1][1], pathlist[0][0], pathlist[0][1]) == [1, 1]):
                        sim.simxSetJointTargetVelocity(
                            clientID_gnd, left_motor, 0.0, sim.simx_opmode_oneshot)
                        sim.simxSetJointTargetVelocity(
                            clientID_gnd, right_motor, 0.0, sim.simx_opmode_oneshot)
                        groundrobotposition = sim.simxGetObjectPosition(
                            clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
                        groundrobotangle_Euler = sim.simxGetObjectOrientation(
                            clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2]
                        groundrobotangle = fun.changeangletoeurrle(
                            groundrobotangle_Euler)
                        del(commands[0])
                    else:
                        sim.simxSetJointTargetVelocity(
                            clientID_gnd, left_motor, 5.0, sim.simx_opmode_oneshot)
                        sim.simxSetJointTargetVelocity(
                            clientID_gnd, right_motor, 5.0, sim.simx_opmode_oneshot)
                        groundrobotposition = sim.simxGetObjectPosition(
                            clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
                        groundrobotangle_Euler = sim.simxGetObjectOrientation(
                            clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2]
                        groundrobotangle = fun.changeangletoeurrle(
                            groundrobotangle_Euler)
                else:
                    errorangle = commands[0][2]-groundrobotangle
                    rotatespeed = errorangle * 0.5
                    rotatespeed = fun.deltaspeed(rotatespeed)
                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, left_motor, 5.0 - rotatespeed, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, right_motor, 5.0 + rotatespeed, sim.simx_opmode_oneshot)
                    groundrobotposition = sim.simxGetObjectPosition(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
                    groundrobotangle_Euler = sim.simxGetObjectOrientation(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2]
                    groundrobotangle = fun.changeangletoeurrle(
                        groundrobotangle_Euler)

        # -------------------------------------- GROUND ROBOT SEARCHES FOR MR YORK --------------------------------------

        if isLookingForBear:
            # Prepare ground robot arms for rescuing Mr. York
            isReadyToSearch = fun.armsMovement(
                clientID_gnd, left_joint, right_joint, isRescueDone)

            tshirt_x = IMG_WIDTH/2.0

            res_gnd, resolution_gnd, image_gnd = sim.simxGetVisionSensorImage(
                clientID_gnd, camera_gnd, 0, sim.simx_opmode_buffer)

            if (res_gnd == sim.simx_return_ok) and isReadyToSearch:
                original = np.array(image_gnd, dtype=np.uint8)
                original.resize([resolution_gnd[0], resolution_gnd[1], 3])
                original = cv2.flip(original, 0)
                original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

                # Find mask for Mr York's tshirt
                tshirt_mask = fun.findBearMask(original)

                values = list(tshirt_mask)
                color_values = np.unique(values)

                # Finding centre of mass of Mr York's tshirt
                tshirt_x, tshirt_y = fun.detectCenterOfMass(tshirt_mask)

                # Look up for Mr York and move towards him
                isRescueDone, isReadyToSearch = states.SEARCHING_BEAR(
                    color_values, tshirt_x, clientID_gnd, left_motor, right_motor, prox_sensor)

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break

else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID_drone)
sim.simxFinish(clientID_gnd)
cv2.destroyAllWindows()
print('Simulation ended')
