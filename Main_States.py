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
def FOLLOWING_PATH(state, path, clientID_gnd, body_gnd, floor, left_motor, right_motor):
    delta = 0.0
    # path = np.delete(path, [0, 2], axis=0)
    print("Starting the path...")
    res_position, position = sim.simxGetObjectPosition(
        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)
    res_orientation, euler_orientation = sim.simxGetObjectOrientation(
        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)

    if (res_position == sim.simx_return_ok) and (res_orientation == sim.simx_return_ok):
        print("Position ready...")
        for command in path:
            position = sim.simxGetObjectPosition(
                clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1]
            pos_x = round(position[0], 4)
            pos_y = round(position[1], 4)
            print("Command position: ", [command[0], command[1]])
            print("Position: ", [pos_x, pos_y])

            if (pos_y != command[0]) or (pos_x != command[1]):
                euler_orientation = sim.simxGetObjectOrientation(
                    clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1]
                orientation_degrees = fun.eulerAnglesToDegrees(
                    round(abs(euler_orientation[2]), 0))

                print("Orientation: ", orientation_degrees)
                print("Command orientation: ", command[2])

                while (orientation_degrees != command[2]):
                    fun.groundMovement(
                        'TURN_LEFT', clientID_gnd, left_motor, right_motor, delta)
                    euler_orientation = sim.simxGetObjectOrientation(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][2]
                    orientation_degrees = round(
                        fun.eulerAnglesToDegrees(euler_orientation), 0)

                # print("Command position: ", [command[0], command[1]])
                # print("Position: ", [pos_x, pos_y])
                # while (pos_y != command[0]) or (pos_x != command[1]):
                #     fun.groundMovement(
                #         'FORWARD', clientID_gnd, left_motor, right_motor, delta)
                #     position = sim.simxGetObjectPosition(
                #         clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1]
                #     pos_x = round(position[0], 4)
                #     pos_y = round(position[1], 4)
                # print("Command position: ", [command[0], command[1]])
                # print("Position: ", [pos_x, pos_y])
