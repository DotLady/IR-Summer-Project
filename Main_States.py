import sim
import cv2
import numpy as np
import Main_Functions as fun
import tcod


IMG_WIDTH = 512
IMG_HEIGHT = 512


def GETTING_MAP(original):
    # cv2.imshow("Drone camera", original)
    print("Creating map...")
    # --------------------------------- DRONE ---------------------------------
    # Find masks for hospital, car and obstacles
    robot_mask, manta_mask, tree_mask, white_obstacle_mask = fun.findColorsMasks(
        original)

    # Find START and END coordinates
    start_x, start_y = fun.detectCenterOfMass(
        robot_mask, False)
    print("Ground robot centre: (", start_x, ", ", start_y, ")")
    end_x, end_y = fun.detectCenterOfMass(
        manta_mask, False)
    print("Red manta's centre: (", end_x, ", ", end_y, ")")

    # Finding a path fron START to END
    obstacles_image = cv2.bitwise_or(tree_mask, white_obstacle_mask)
    thick_mask = fun.createFatMap(white_obstacle_mask)
    map_matrix = cv2.bitwise_or(tree_mask, thick_mask)

    # Path Finding algorithm
    aStar_path = fun.aStar(map_matrix, start_y, start_x, end_y, end_x)

    path_image = fun.pathToImage(obstacles_image, aStar_path)

    commands = fun.getCommands(aStar_path)

    # cv2.imshow("FatImage", map_matrix)
    # # cv2.imshow("PathFinder", pf_path_image)
    # cv2.imshow("A Star", path_image)

    # Save map and path images
    # map_matrix.dtype = 'uint8'
    # path_image.dtype = 'uint8'
    # status_map = cv2.imwrite(
    #     'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/Map_maze.jpg', map_matrix)
    # status_path = cv2.imwrite(
    #     'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/Path_maze.jpg', path_image)
    # print("Map image saved status: ", status_map)
    # print("Path image saved status: ", status_path)

    if (len(commands) != 0):
        return path_image, commands, True
    else:
        return path_image, commands, False


def SEARCHING_BEAR():
    # Find mask for Mr York's tshirt
    tshirt_mask = fun_gnd.findBearMask(original)
    # cv2.imshow("Mask", tshirt_mask)
    values = list(tshirt_mask)
    color_values = np.unique(values)

    # Find START and END coordinates
    tshirt_image, tshirt_x, tshirt_y = fun.detectCenterOfMass(
        tshirt_mask, True)
    cv2.imshow("Center_of_Tshirt", tshirt_image)

    delta = fun.controllerMove(tshirt_x)
    return color_values, delta


# ---------------------------- NOT READY ----------------------------
def FOLLOWING_PATH(state, path, clientID_gnd, body_gnd, floor, leftMotor, rightMotor):
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
                        'TURN_LEFT', clientID_gnd, leftMotor, rightMotor, delta)
                    euler_orientation = sim.simxGetObjectOrientation(
                        clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1][2]
                    orientation_degrees = round(
                        fun.eulerAnglesToDegrees(euler_orientation), 0)

                # print("Command position: ", [command[0], command[1]])
                # print("Position: ", [pos_x, pos_y])
                # while (pos_y != command[0]) or (pos_x != command[1]):
                #     fun.groundMovement(
                #         'FORWARD', clientID_gnd, leftMotor, rightMotor, delta)
                #     position = sim.simxGetObjectPosition(
                #         clientID_gnd, body_gnd, floor, sim.simx_opmode_oneshot)[1]
                #     pos_x = round(position[0], 4)
                #     pos_y = round(position[1], 4)
                # print("Command position: ", [command[0], command[1]])
                # print("Position: ", [pos_x, pos_y])
