#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: Daniela Ordóñez Tapia

"""

import sim
import cv2
import numpy as np
import matplotlib as mpl
import Quadcopter_Functions as fun
from matplotlib import pyplot as plt
from matplotlib import colors
import tcod

IMG_WIDTH = 512
IMG_HEIGHT = 512
# GRND_BOT_SIZE = 1

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
    res, camera_orth = sim.simxGetObjectHandle(
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

    res, resolution, image_orth = sim.simxGetVisionSensorImage(
        clientID, camera_orth, 0, sim.simx_opmode_streaming)

    while (sim.simxGetConnectionId(clientID) != -1):
        # Get image from Camera
        res, resolution, image_orth = sim.simxGetVisionSensorImage(
            clientID, camera_orth, 0, sim.simx_opmode_buffer)

        if res == sim.simx_return_ok:
            original = np.array(image_orth, dtype=np.uint8)
            original.resize([resolution[0], resolution[1], 3])
            original = cv2.flip(original, 0)
            original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
            # cv2.imshow("Camera", original)

            # Find masks for hospital, car and obstacles
            hospital_mask, car_mask, tree_mask, white_obstacle_mask = fun.findColorsMasks(
                original)

            # Find START and END coordinates
            center_hospital_image, start_x, start_y = fun.detectCenterOfMass(
                hospital_mask, False)
            print("Centro hospital: (", start_x, ", ", start_y, ")")
            center_car_image, end_x, end_y = fun.detectCenterOfMass(
                car_mask, False)
            print("Centro auto: (", end_x, ", ", end_y, ")")

            # output_image = cv2.bitwise_and(original, original, mask=map_mask)
            # cv2.imshow("HospitalMask", center_hospital_image)
            # cv2.imshow("CarMask", center_car_image)

            # Finding a path fron START to END
            obstacles_image = cv2.bitwise_or(tree_mask, white_obstacle_mask)
            thick_mask = fun.createFatMap(white_obstacle_mask)
            map_matrix = cv2.bitwise_or(tree_mask, thick_mask)

            # # Save map image
            # map_matrix.dtype = 'uint8'
            # status = cv2.imwrite(
            #     'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/python_maze.jpg', map_matrix)
            # print("Image written to file-system : ", status)

            # Path Finding algorithms
            pf_path = fun.pathFinder(
                map_matrix, start_y, start_x, end_y, end_x)

            aStar_path = fun.aStar(map_matrix, start_y, start_x, end_y, end_x)

            path_image = fun.pathToImage(obstacles_image, aStar_path)

            commands = fun.getCommands(aStar_path)

            commands_meters = fun.pixelsToMeters(commands)

            cv2.imshow("FatImage", map_matrix)
            # cv2.imshow("PathFinder", pf_path_image)

            cv2.imshow("A Star", path_image)

            # ------------------------------------- END TESTING -------------------------------------

            corners_image = fun.detectCorners(white_obstacle_mask)
            # cv2.imshow("Corners", corners_image)

            contours_image = fun.detectContours(white_obstacle_mask)
            # im_with_keypoints = detectBlobs(obstacle_mask)
            # Show keypoints
            # cv2.imshow("Contours", contours_image)
            # cv2.waitKey(0)

        elif res == sim.simx_return_novalue_flag:
            # Camera has not started or is not returning images
            print("Wait, there's no image yet")
        else:
            # Something else has happened
            print("Unexpected error returned", res)

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break

        # res, position = sim.simxGetObjectPosition(
        #     clientID, body, floor, sim.simx_opmode_oneshot)
        # if res == sim.simx_return_ok:
        #     print("X: ", round(position[0], 0))
        #     print("Y: ", round(position[1], 0))
        #     print("Z: ", round(position[2], 0))

else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
