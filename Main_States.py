import sim
import cv2
import numpy as np
import Main_Functions as fun
import tcod
import time


IMG_WIDTH = 512
IMG_HEIGHT = 512


def GETTING_MAP(original):
    # cv2.imshow("Drone camera", original)
    print("Creating map...")

    aStar_path = []

    while(len(aStar_path) == 0):
        # Find masks for hospital, car and obstacles
        robot_mask, manta_mask, tree_mask, white_obstacle_mask = fun.findColorsMasks(
            original)

        # Find START and END coordinates
        start_x, start_y = fun.detectCenterOfMass(robot_mask)
        # print("Ground robot centre: (", start_x, ", ", start_y, ")")
        temp_x, end_y = fun.detectCenterOfMass(manta_mask)
        end_x = temp_x-10
        # print("Red manta's centre: (", end_x, ", ", end_y, ")")

        # Finding a path fron START to END
        obstacles_image = cv2.bitwise_or(tree_mask, white_obstacle_mask)
        cv2.imshow("ANNOYING1", obstacles_image)
        cv2.waitKey(0)

        thick_mask = fun.createFatMap(white_obstacle_mask, 18)
        thick_tree = fun.createFatMap(tree_mask, 10)
        map_matrix = cv2.bitwise_or(thick_tree, thick_mask)

        # # Save map and path image
        map_matrix.dtype = 'uint8'
        status_path = cv2.imwrite(
            'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/Thick_maze.jpg', map_matrix)

        # Path Finding algorithm
        aStar_path = fun.aStar(map_matrix, start_y, start_x, end_y, end_x)
        # aStar_path = fun.pathFinder(map_matrix, start_y, start_x, end_y, end_x)

    path_image = fun.pathToImage(obstacles_image, aStar_path)

    commands = fun.getCommands(aStar_path)

    if (len(commands) != 0):
        return commands, True
    else:
        return commands, False


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
