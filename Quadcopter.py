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

IMG_WIDTH = 512
IMG_HEIGHT = 512

grid_matrix = np.zeros((512, 512))

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
     print('Obtaining handles of simulation objects...')

      # Floor orthographic camera for exploration
      res, camera = sim.simxGetObjectHandle(
           clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
       if res != sim.simx_return_ok:
            print('Could not get handle to Camera')

        # Body
        res, body = sim.simxGetObjectHandle(
            clientID, 'Quadricopter_target', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to Robot')

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

            commands = []

            # State Machine
            # main_fun.currentState('GETTING_MAP', res, resolution, image_orth)

            if res == sim.simx_return_ok:
                original = np.array(image, dtype=np.uint8)
                original.resize([resolution[0], resolution[1], 3])
                original = cv2.flip(original, 0)
                original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
                # cv2.imshow("Camera", original)

                # Find masks for hospital, car and obstacles
                robot_mask, manta_mask, tree_mask, white_obstacle_mask = fun.findColorsMasks(
                    original)

                # Find START and END coordinates
                start_x, start_y = fun.detectCenterOfMass(robot_mask, False)
                print("Ground robot centre: (", start_x, ", ", start_y, ")")
                end_x, end_y = fun.detectCenterOfMass(manta_mask, False)
                print("Red manta's centre: (", end_x, ", ", end_y, ")")

                # Finding a path fron START to END
                obstacles_image = cv2.bitwise_or(
                    tree_mask, white_obstacle_mask)
                thick_mask = fun.createFatMap(white_obstacle_mask)
                map_matrix = cv2.bitwise_or(tree_mask, thick_mask)
                # cv2.imshow("FatImage", map_matrix)

                # Path Finding algorithms
                pf_path = fun.pathFinder(
                    map_matrix, start_y, start_x, end_y, end_x)

                aStar_path = fun.aStar(
                    map_matrix, start_y, start_x, end_y, end_x)

                path_image = fun.pathToImage(obstacles_image, aStar_path)
                cv2.imshow("A Star", path_image)

                commands = fun.getCommands(aStar_path)

                # Save map and path images
                # map_matrix.dtype = 'uint8'
                # path_image.dtype = 'uint8'
                # status_map = cv2.imwrite(
                #     'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/Map_maze.jpg', map_matrix)
                # status_path = cv2.imwrite(
                #     'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/Path_maze.jpg', path_image)
                # print("Map image saved status: ", status_map)
                # print("Path image saved status: ", status_path)

                # ------------------------------------- TESTING -------------------------------------

                # When we CANNOT IDENTIFY the red Manta

                # corners_image = fun.detectCorners(white_obstacle_mask)
                # cv2.imshow("Corners", corners_image)

                # contours_image = fun.detectContours(white_obstacle_mask)
                # im_with_keypoints = detectBlobs(obstacle_mask)
                # Show keypoints
                # cv2.imshow("Contours", contours_image)
                # cv2.waitKey(0)

                # ------------------------------------- TESTING -------------------------------------

            elif res == sim.simx_return_novalue_flag:
                # Camera has not started or is not returning images
                print("Wait, there's no image yet")
            else:
                # Something else has happened
                print("Unexpected error returned", res)

            keypress = cv2.waitKey(1) & 0xFF
            if keypress == ord('q'):
                break

        else:
            print('Could not connect to remote API server')

        # Close all simulation elements
        sim.simxFinish(clientID)
        cv2.destroyAllWindows()
        print('Simulation ended')
