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
FORWARD_SPEED = 12.0
TURNING_SPEED = 5.0

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

    leftWheelDiameter = sim.simxGetObjectFloatParameter(clientID_gnd, leftWheel, 18, sim.simx_opmode_oneshot)[1] - sim.simxGetObjectFloatParameter(clientID_gnd,
                                                                                                                                                   leftWheel, 15, sim.simx_opmode_oneshot)[1]

    isDroneCentered = False
    isMapReady = False
    isPreparedToGo = False
    isMovingToManta = False
    isLookingForBear = False
    isRescueDone = False
    isReadyToSearch = False
    isGoingBackToManta = False
    isReturningMapReady = False
    isWaitingForMap = False
    isGoingBackToHospital = False

    delta = 0.0
    commands = []

    manta_coordinates = []
    returning_commands = []
    color_values = []

    while (sim.simxGetConnectionId(clientID_drone) != -1) and (sim.simxGetConnectionId(clientID_gnd) != -1):

        # -------------------------------------- DRONE MOVES TO THE CENTRE OF THE SCENE --------------------------------------

        # Moving drone to the center of the environment
        if (not isDroneCentered):
            drone_viewposition, repeatseed, isDroneCentered = fun.droneInitialMovement(
                clientID_drone, drone_base_handle, drone_target_handle, floor, drone_viewposition, repeatseed)

        # If we don't have a path
        if (not isMapReady) and isDroneCentered:
            time.sleep(40)
            while (len(commands) == 0):
                res_drone, resolution_drone, image_drone = sim.simxGetVisionSensorImage(
                    clientID_drone, camera_drone, 0, sim.simx_opmode_buffer)

                if res_drone == sim.simx_return_ok:
                    # time.sleep(40)
                    original = np.array(image_drone, dtype=np.uint8)
                    original.resize(
                        [resolution_drone[0], resolution_drone[1], 3])
                    original = cv2.flip(original, 0)
                    original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

                    # Image processing with the drone's camera. Getting a path with A* algorithm
                    commands, isMapReady = states.GETTING_MAP(
                        original, 'MAP_TO_RESCUE')

            if isMapReady:
                print("\nPath is ready!")
                print(commands)
                manta_coordinates = [commands[-1]]
                isMovingToManta = True

        # -------------------------------------- GROUND ROBOT MOVES TOWARDS RED MANTA --------------------------------------

        if isMapReady and isMovingToManta:  # and isPreparedToGo:
            gnd_robot_position = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
            gnd_robot_angle = fun.changeRadiansToDegrees(sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])

            # If get a path list
            if (len(commands) >= 1):
                # Check if the angle of ground robot is same as the list, if true check position else robot rotating
                if (fun.checkAngle(gnd_robot_angle, commands[0][2]) == True):
                    # Function to check position, [x,y] x y are bool values, 1 means two positions are same, 0 means different
                    # will return [1,1] when arrive the position.
                    if (fun.checkPosition(gnd_robot_position[1][0], gnd_robot_position[1][1], commands[0][0], commands[0][1]) == [1, 1]):
                        fun.groundMovement(
                            'STOP', clientID_gnd, left_motor, right_motor, 0.0)
                        commands = commands[1:]

                    else:
                        fun.groundMovement(
                            'FORWARD', clientID_gnd, left_motor, right_motor, FORWARD_SPEED)

                else:
                    error_angle = commands[0][2] - gnd_robot_angle
                    rotate_speed = fun.deltaSpeed(error_angle*0.1)
                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, left_motor, 1.0 - rotate_speed, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, right_motor, 1.0 + rotate_speed, sim.simx_opmode_oneshot)

            else:
                isMovingToManta = False
                isLookingForBear = True

        # -------------------------------------- GROUND ROBOT SEARCHES FOR MR YORK --------------------------------------

        if isLookingForBear and (not isMovingToManta):
            # Prepare ground robot arms for rescuing Mr. York
            isReadyToSearch, armsClosed = fun.armsMovement(
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

            if armsClosed:
                isGoingBackToManta = True
                isLookingForBear = False
                print("Let's go back...")

        if isGoingBackToManta and armsClosed:

            gnd_robot_position = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
            gnd_robot_angle = fun.changeRadiansToDegrees(sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])

            desired_y = manta_coordinates[0][0]
            desired_x = manta_coordinates[0][1]
            current_y = gnd_robot_position[1][0]
            current_x = gnd_robot_position[1][1]

            desired_angle = fun.changeRadiansToDegrees(np.arctan2(
                (desired_x-current_x), (desired_y-current_y)))
            error_angle = gnd_robot_angle - desired_angle

            # Check for the correct orientation of the ground robot
            while(abs(error_angle) > 10.0):
                if error_angle < 0.0:
                    fun.groundMovement(
                        'TURN_LEFT', clientID_gnd, left_motor, right_motor, TURNING_SPEED)
                else:
                    fun.groundMovement(
                        'TURN_RIGHT', clientID_gnd, left_motor, right_motor, TURNING_SPEED)

                gnd_robot_angle = fun.changeRadiansToDegrees(sim.simxGetObjectOrientation(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][2])
                error_angle = gnd_robot_angle - desired_angle

            # Check for the correct position of the ground robot
            current_x = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][1]
            position_x_diff = abs(current_x - desired_x)
            while (position_x_diff > 0.2):
                fun.groundMovement(
                    'FORWARD', clientID_gnd, left_motor, right_motor, FORWARD_SPEED)
                current_x = sim.simxGetObjectPosition(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][1]
                position_x_diff = abs(current_x - desired_x)

            fun.groundMovement(
                'STOP', clientID_gnd, left_motor, right_motor, 0.0)

            print("Waiting for returning map...")
            isWaitingForMap = True
            isGoingBackToManta = False

        # If we don't have a path
        if (not isReturningMapReady) and isWaitingForMap:
            res_drone, resolution_drone, image_drone = sim.simxGetVisionSensorImage(
                clientID_drone, camera_drone, 0, sim.simx_opmode_buffer)

            if res_drone == sim.simx_return_ok:
                original = np.array(image_drone, dtype=np.uint8)
                original.resize([resolution_drone[0], resolution_drone[1], 3])
                original = cv2.flip(original, 0)
                original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

                # Image processing with the drone's camera. Getting a path with A* algorithm
                returning_commands, isReturningMapReady = states.GETTING_MAP(
                    original, 'MAP_TO_RETURN')

                if isMapReady:
                    print("\nReturning path is ready!")
                    isReturningMapReady = True
                    isWaitingForMap = False
                    isGoingBackToHospital = True

        if isReturningMapReady and isGoingBackToHospital:
            gnd_robot_position = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
            gnd_robot_angle = fun.changeRadiansToDegrees(sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])

            # If get a path list
            if (len(returning_commands) >= 1):
                # Check if the angle of ground robot is same as the list, if true check position else robot rotating
                if (fun.checkAngle(gnd_robot_angle, returning_commands[0][2]) == True):
                    # Function to check position, [x,y] x y are bool values, 1 means two positions are same, 0 means different
                    # will return [1,1] when arrive the position.
                    if (fun.checkPosition(gnd_robot_position[1][0], gnd_robot_position[1][1], returning_commands[0][0], returning_commands[0][1]) == [1, 1]):
                        fun.groundMovement(
                            'STOP', clientID_gnd, left_motor, right_motor, 0.0)

                        returning_commands = returning_commands[1:]

                    else:
                        fun.groundMovement(
                            'FORWARD', clientID_gnd, left_motor, right_motor, FORWARD_SPEED)

                else:
                    error_angle = returning_commands[0][2] - gnd_robot_angle
                    rotate_speed = fun.deltaSpeed(error_angle*0.1)

                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, left_motor, 1.0 - rotate_speed, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(
                        clientID_gnd, right_motor, 1.0 + rotate_speed, sim.simx_opmode_oneshot)
            else:
                isGoingBackToHospital = False
                print("Mr York is safe now!!")
                fun.groundMovement('STOP', clientID_gnd,
                                   left_motor, right_motor, 0.0)

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
