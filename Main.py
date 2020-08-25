import sim
import cv2
import numpy as np
import Main_Functions as fun
import Main_States as states
import tcod
import time

IMG_WIDTH = 512
IMG_HEIGHT = 512
ONE_UNIT_DISTANCE = 6.283185307179586
FORWARD_SPEED = 15.0

drone_viewposition = [0, 0, 8]
repeatseed = 0
grid_matrix = np.zeros((512, 512))

lastLeftWheelPosition = 0.0
lastRightWheelPosition = 0.0
leftWheelOdom = 0.0

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

    isDroneCentered = False
    isMapReady = False
    isMovingToManta = False
    isLookingForBear = False
    isRescueDone = False
    isReadyToSearch = False
    isGoingBackToManta = False
    isGoingBackToHospital = False

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
                    manta_coordinates = commands[-1]
                    isMovingToManta = True
                    isLookingForBear = False

        # -------------------------------------- GROUND ROBOT MOVES TOWARDS RED MANTA --------------------------------------

        if isMapReady and isMovingToManta:  # and isPreparedToGo:
            # isMovingToManta, isLookingForBear = states.FOLLOWING_PATH(
            #     state, commands, clientID_gnd, body_gnd, floor, left_motor, right_motor)

            isLookingForBear = states.MOVING_TO_PATH(commands, clientID_gnd, body_gnd, floor, leftWheel,
                                                     left_motor, right_motor, lastLeftWheelPosition, lastRightWheelPosition)

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

                res_left_joint, left_joint_pos = sim.simxGetJointPosition(
                    clientID_gnd, left_joint, sim.simx_opmode_oneshot)

                if res_left_joint == sim.simx_return_ok:
                    if (left_joint_pos > 0.0) and isRescueDone:
                        print("HELLO")
                        isGoingBackToManta = True
                        # isLookingForBear = False

        if isGoingBackToManta:
            # fun.getReturnAngle(manta_coordinates):
            current_position = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1]
            current_orientation = fun.eulerAnglesToDegrees(sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][2])

            desired_y = manta_coordinates[0]
            desired_x = manta_coordinates[1]
            current_y = current_position[0]
            current_x = current_position[1]

            desired_orientation = numpy.arctan2(
                (desired_x-current_x), (desired_y-current_y))*180.0/numpy.pi
            angle_diff = current_orientation - desired_orientation

            while(abs(angle_diff) > 8.0):
                if angle_diff < 0.0:
                    fun.groundMovement(
                        'TURN_LEFT', clientID_gnd, left_motor, right_motor, delta)
                else:
                    fun.groundMovement(
                        'TURN_RIGHT', clientID_gnd, left_motor, right_motor, delta)

            position_x_diff = abs(current_x - desired_x)
            while (position_x_diff > 0.2):
                fun.groundMovement(
                    'FORWARD', clientID_gnd, left_motor, right_motor, delta)
                current_x = round(sim.simxGetObjectPosition(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][1], 2)
                position_x_diff = abs(current_x - desired_x)

            fun.groundMovement('STOP', clientID_gnd,
                               left_motor, right_motor, 0.0)
            isGoingBackToHospital = True

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
