#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: Daniela Ordóñez Tapia

"""

import sim
import cv2
import numpy as np
import matplotlib as mpl
import Main_Functions as main_fun
import Quadcopter_Functions as fun
import multiprocessing as mp
import tcod
import time

IMG_WIDTH = 512
IMG_HEIGHT = 512

grid_matrix = np.zeros((512, 512))
drone_viewposition = [0, 0, 8]
repeatseed = 0

# -------------------------------------- START PROGRAM --------------------------------------
# Start Program and just in case, close all opened connections
print('Program started')
sim.simxFinish(-1)

# Connect to simulator running on localhost
# V-REP runs on port 19997, a script opens the API on port 19999
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

# Connect to the simulation
if clientID != -1:
    print('Connected to remote API server')

    # Get handles to simulation objects
    print('Obtaining handles of simulation objects')

    # Floor orthographic camera for exploration
    res, camera = sim.simxGetObjectHandle(
        clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    res, floor = sim.simxGetObjectHandle(
        clientID, 'ResizableFloor_5_25', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get hanlde of floor')

    res, drone_base_hanlde = sim.simxGetObjectHandle(
        clientID, 'Quadricopter_base', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get hanlde of drone base')

    res, drone_target_hanlde = sim.simxGetObjectHandle(
        clientID, 'Quadricopter_target', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get hanlde of drone target')

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

        # sim.simxSetJointTargetVelocity(
        #     clientID, leftMotor, 1.0, sim.simx_opmode_oneshot)
        # sim.simxSetJointTargetVelocity(
        #     clientID, rightMotor, 1.0, sim.simx_opmode_oneshot)
        # # Get image from CameraAa
        # res_pers, resolution, image_pers = sim.simxGetVisionSensorImage(
        #     clientID, camera_pers, 0, sim.simx_opmode_buffer)
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_buffer)

        # # get drone position
        # drone_base_position = sim.simxGetObjectPosition(
        #     clientID, drone_base_hanlde, floor, sim.simx_opmode_blocking)
        # drone_target_position = sim.simxGetObjectPosition(
        #     clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        # print(drone_base_position)

        # # Drone move in z axis
        # if(drone_base_position[1][2] <= 8 and repeatseed == 0):
        #     repeatseed = 1
        #     for i in range(int(drone_base_position[1][2]+1), 9):
        #         drone_base_position = sim.simxGetObjectPosition(
        #             clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        #         sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [
        #                                   drone_base_position[1][0], drone_base_position[1][1], i], sim.simx_opmode_blocking)
        #         print(drone_base_position)
        #         time.sleep(2)
        # # Drone move in x axis
        # if(drone_base_position[1][0] != 0 and repeatseed == 1):
        #     repeatseed = 2
        #     drone_x_sign = drone_base_position[1][0] / \
        #         abs(drone_base_position[1][0])
        #     for i in range(1, ((int(abs(drone_base_position[1][0])))*10)+1):
        #         drone_base_position = sim.simxGetObjectPosition(
        #             clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        #         sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [drone_base_position[1][0] - drone_x_sign*0.1
        #         sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [
        #                                   drone_base_position[1][0] - drone_x_sign*0.05, drone_base_position[1][1], drone_base_position[1][2]], sim.simx_opmode_blocking)
        #         print(drone_base_position)
        #         time.sleep(0.1)
        #     time.sleep(6)
        #     drone_base_position=sim.simxGetObjectPosition(
        #         clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        #     print(drone_base_position)

        # if(drone_base_position[1][0] != 0 and repeatseed == 2):
        #     repeatseed=3
        #     drone_y_sign=drone_base_position[1][1] /
        #         abs(drone_base_position[1][1])
        #     for i in range(1, ((int(abs(drone_base_position[1][1])))*20)+1):
        #         drone_base_position=sim.simxGetObjectPosition(
        #             clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        #         sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [
        #                                   drone_base_position[1][0], drone_base_position[1][1] - drone_y_sign*0.05, drone_base_position[1][2]], sim.simx_opmode_blocking)
        #         print(drone_base_position)
        #         time.sleep(0.1)
        #     time.sleep(6)
        #     drone_base_position=sim.simxGetObjectPosition(
        #         clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        #     print(drone_base_position)
        #     print(drone_target_position)
        # image process
        if res == sim.simx_return_ok:
            original = np.array(image, dtype=np.uint8)
            original.resize([resolution[0], resolution[1], 3])
            original = cv2.flip(original, 0)
            original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
            # cv2.imshow("Camera", original)

            robot_mask, tree_mask, white_obstacle_mask = main_fun.findColorsMasks(
                original)

            obstacles_image = cv2.bitwise_or(tree_mask, white_obstacle_mask)
            # cv2.imshow("ANNOYING1", obstacles_image)
            # cv2.waitKey(0)

            thick_mask = main_fun.createFatMap(white_obstacle_mask, 18)
            thick_tree = main_fun.createFatMap(tree_mask, 10)
            map_matrix = cv2.bitwise_or(thick_tree, thick_mask)

            # Find START and END coordinates
            start_x, start_y = main_fun.detectCenterOfMass(robot_mask)
            cv2.imshow("ROBOT", robot_mask)
            cv2.waitKey(0)
            manta_mask = main_fun.findMantaMask(original)
            end_x, temp_y = main_fun.detectCenterOfMass(manta_mask)
            end_y = temp_y

            # aStar_path = main_fun.aStar(map_matrix, start_y, start_x, end_y, end_x)
            cost_matrix = np.ones((IMG_HEIGHT, IMG_WIDTH), dtype=np.int8)

            for i in range(len(map_matrix)):
                for j in range(len(map_matrix[0])):
                    if map_matrix[i][j] == 255:
                        cost_matrix[i][j] = 0

            print("Finding AStar path...")

            aStar_graph = tcod.path.AStar(cost_matrix, 1.0)
            print("POINTS: ", start_y, start_x, end_y, end_x)

            path_list = aStar_graph.get_path(start_y, start_x, end_y, end_x)
            # print("Length commands: ", len(path_list))
            # aStar_path = fun.pathFinder(map_matrix, start_y, start_x, end_y, end_x)

            path_image = main_fun.pathToImage(obstacles_image, path_list)
            cv2.imshow("Path_image", path_image)
            commands = main_fun.getCommands(path_list)

        elif res == sim.simx_return_novalue_flag:
            # Camera has not started or is not returning images
            print("Wait, there's no image yet")
        else:
            print("Path is ready!!!")

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break

        # drone move to center


else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
