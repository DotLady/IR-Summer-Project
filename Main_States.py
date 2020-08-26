import sim
import cv2
import numpy as np
import Main_Functions as fun
import tcod
import time


IMG_WIDTH = 512
IMG_HEIGHT = 512
ONE_UNIT_DISTANCE = 6.283185307179586
FORWARD_SPEED = 3.0


def GETTING_MAP(original, text):
    # cv2.imshow("Drone camera", original)
    print("Creating map...")

    aStar_path = []

    while(len(aStar_path) == 0):
        # Find masks for hospital, car and obstacles
        robot_mask, tree_mask, white_obstacle_mask = fun.findColorsMasks(
            original)

        obstacles_image = cv2.bitwise_or(tree_mask, white_obstacle_mask)
        cv2.imshow("ANNOYING1", obstacles_image)
        cv2.waitKey(0)

        thick_mask = fun.createFatMap(white_obstacle_mask, 18)
        thick_tree = fun.createFatMap(tree_mask, 10)
        map_matrix = cv2.bitwise_or(thick_tree, thick_mask)

        # Find START and END coordinates
        start_x, start_y = fun.detectCenterOfMass(robot_mask)
        # print("Ground robot centre: (", start_x, ", ", start_y, ")")

        if (text == 'MAP_TO_RESCUE'):
            manta_mask = fun.findMantaMask(original)
            end_x, temp_y = fun.detectCenterOfMass(manta_mask)
            end_y = temp_y+20
            # print("Red manta's centre: (", end_x, ", ", end_y, ")")
        else:
            hospital_mask = fun.findHospitalMask(original)
            end_x, end_y = fun.detectCenterOfMass(hospital_mask)
            # print("Hospital's centre: (", end_x, ", ", end_y, ")")

        # Save map image
        # map_matrix.dtype = 'uint8'
        # status_path = cv2.imwrite(
        #     'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/Thick_maze.jpg', map_matrix)

        # Path Finding algorithm
        aStar_path = fun.aStar(map_matrix, start_y, start_x, end_y, end_x)
        # aStar_path = fun.pathFinder(map_matrix, start_y, start_x, end_y, end_x)

    path_image = fun.pathToImage(obstacles_image, aStar_path)

    commands = fun.getCommands(aStar_path)
    # print("Type of commands: ", type(commands))

    if (len(commands) != 0):
        return commands, end_x, end_y, True
    else:
        return commands, 0.0, 0.0, False


def MOVING_TO_PATH(commands, clientID_gnd, body_gnd, floor, leftWheel, left_motor, right_motor, lastLeftWheelPosition, lastRightWheelPosition):

    leftWheelDiameter = sim.simxGetObjectFloatParameter(clientID_gnd, leftWheel, 18, sim.simx_opmode_oneshot)[1] \
        - sim.simxGetObjectFloatParameter(clientID_gnd,
                                          leftWheel, 15, sim.simx_opmode_oneshot)[1]

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
    # if -6.0<=gnd_robot_position[1][0] <=-5.9:
    #     sim.simxSetJointTargetVelocity(
    #         clientID, left_motor, 0.0, sim.simx_opmode_oneshot)
    #     sim.simxSetJointTargetVelocity(
    #         clientID, right_motor, 0.0, sim.simx_opmode_oneshot)
    #
    # Update the position
    gnd_robot_position = sim.simxGetObjectPosition(
        clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
    gnd_robot_angle = fun.changeAngleToEuler(sim.simxGetObjectOrientation(
        clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])
    # print(gnd_robot_angle)

    # If get a path list
    if (len(commands) >= 1):
        # Check if the angle of ground robot is same as the list, if true check position else robot rotating
        print(commands[0])
        if (fun.checkAngle(gnd_robot_angle, commands[0][2]) == True):
            sim.simxSetJointTargetVelocity(
                clientID_gnd, left_motor, 0.0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID_gnd, right_motor, 0.0, sim.simx_opmode_oneshot)

            gnd_robot_position = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
            gnd_robot_angle = fun.changeAngleToEuler(sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])

            # Function to check position, [x,y] x y are bool values, 1 means two positions are same, 0 means different
            # will return [1,1] when arrive the position.
            if (fun.checkPosition(gnd_robot_position[1][0], gnd_robot_position[1][1], commands[0][0], commands[0][1]) == [1, 1]):
                sim.simxSetJointTargetVelocity(
                    clientID_gnd, left_motor, 0.0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID_gnd, right_motor, 0.0, sim.simx_opmode_oneshot)

                gnd_robot_position = sim.simxGetObjectPosition(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
                gnd_robot_angle = fun.changeAngleToEuler(sim.simxGetObjectOrientation(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])

                commands = commands[1:]

            else:
                sim.simxSetJointTargetVelocity(
                    clientID_gnd, left_motor, 3.0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID_gnd, right_motor, 3.0, sim.simx_opmode_oneshot)

                gnd_robot_position = sim.simxGetObjectPosition(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
                gnd_robot_angle = fun.changeAngleToEuler(sim.simxGetObjectOrientation(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])

        else:
            error_angle = commands[0][2]-gnd_robot_angle
            rotate_speed = fun.deltaSpeed(error_angle * 0.5)

            sim.simxSetJointTargetVelocity(
                clientID_gnd, left_motor, 1.0 - rotate_speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID_gnd, right_motor, 1.0 + rotate_speed, sim.simx_opmode_oneshot)

            gnd_robot_position = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)
            gnd_robot_angle = fun.changeAngleToEuler(sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_blocking)[1][2])

        return False, True

    else:
        return True, False


def SEARCHING_BEAR(color_values, tshirt_x, clientID, left_motor, right_motor, prox_sensor):
    # Function returns isRescueDone, isReadyToSearch
    # Calculated offset
    delta = fun.controllerMove(tshirt_x)

    # Look up for Mr York and move towards him
    if (len(color_values) == 1):
        if (color_values[0] == 0):
            fun.groundMovement(
                'TURN_RIGHT', clientID, left_motor, right_motor, delta)

        elif (color_values[0] == 255):
            fun.groundMovement(
                'STOP', clientID, left_motor, right_motor, delta)
            return True, False

    else:
        fun.groundMovement('FORWARD', clientID,
                           left_motor, right_motor, delta)
        isDetecting = sim.simxReadProximitySensor(
            clientID, prox_sensor, sim.simx_opmode_oneshot)[1]
        if isDetecting:
            fun.groundMovement(
                'STOP', clientID, left_motor, right_motor, delta)
            return True, False

    return False, True


# ---------------------------- NOT READY ----------------------------
def FOLLOWING_PATH(state, commands, clientID_gnd, body_gnd, floor, left_motor, right_motor):
    res_position, current_position = sim.simxGetObjectPosition(
        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)
    res_orientation, euler_orientation = sim.simxGetObjectOrientation(
        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)

    if (res_position == sim.simx_return_ok) and (res_orientation == sim.simx_return_ok):
        print("Moving towards red manta...")
        for command in commands:
            # Check for the correct orientation of the ground robot
            current_orientation = fun.eulerAnglesToDegrees(sim.simxGetObjectOrientation(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][2])
            desired_orientation = command[2]
            angle_diff = current_orientation - desired_orientation

            while(abs(angle_diff) > 8.0):
                if angle_diff < 0.0:
                    fun.groundMovement(
                        'TURN_LEFT', clientID_gnd, left_motor, right_motor, delta)
                else:
                    fun.groundMovement(
                        'TURN_RIGHT', clientID_gnd, left_motor, right_motor, delta)

                current_orientation = fun.eulerAnglesToDegrees(sim.simxGetObjectOrientation(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][2])
                angle_diff = current_orientation - desired_orientation

            # Check for the correct position of the ground robot
            if (abs(desired_orientation) == 90.0):
                current_x = round(sim.simxGetObjectPosition(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][1], 2)
                desired_x = command[1]
                position_x_diff = abs(current_x - desired_x)
                while (position_x_diff > 0.2):
                    fun.groundMovement(
                        'FORWARD', clientID_gnd, left_motor, right_motor, delta)
                    current_x = round(sim.simxGetObjectPosition(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][1], 2)
                    position_x_diff = abs(current_x - desired_x)
                    # print("X difference: ", position_x_diff)
            else:
                current_y = round(sim.simxGetObjectPosition(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][0], 2)
                desired_y = command[0]
                position_y_diff = abs(current_y - desired_y)
                while (position_y_diff > 0.2):
                    fun.groundMovement(
                        'FORWARD', clientID_gnd, left_motor, right_motor, delta)
                    current_y = round(sim.simxGetObjectPosition(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][0], 2)
                    position_y_diff = abs(current_y - desired_y)
                    # print("Y difference: ", position_y_diff)
    return False, True
