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
    print('Obtaining handles of simulation objects')

    # Floor perspective camera for exploration
    res, camera_pers = sim.simxGetObjectHandle(
        clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Floor orthographic camera for exploration
    res, camera_orth = sim.simxGetObjectHandle(
        clientID, 'Vision_sensor0', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')


# -------------------------------------- Main Control Loop --------------------------------------

    # Start main control loop
    print('Starting control loop')

    res, resolution, image_pers = sim.simxGetVisionSensorImage(
        clientID, camera_pers, 0, sim.simx_opmode_streaming)
    res, resolution, image_orth = sim.simxGetVisionSensorImage(
        clientID, camera_orth, 0, sim.simx_opmode_streaming)

    while (sim.simxGetConnectionId(clientID) != -1):
        # Get image from Camera
        res_pers, resolution, image_pers = sim.simxGetVisionSensorImage(
            clientID, camera_pers, 0, sim.simx_opmode_buffer)
        res, resolution, image_orth = sim.simxGetVisionSensorImage(
            clientID, camera_orth, 0, sim.simx_opmode_buffer)

        if res == sim.simx_return_ok:
            original = np.array(image_orth, dtype=np.uint8)
            original.resize([resolution[0], resolution[1], 3])
            original = cv2.flip(original, 0)
            original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
            # cv2.imshow("Camera", original)

            map_mask, tree_mask, white_mask = fun.findColorsMasks(original)
            obstacle_mask = cv2.bitwise_or(tree_mask, white_mask)

            output_image = cv2.bitwise_and(
                original, original, mask=map_mask)
            # cv2.imshow("MapMask", output_image)

            obstacle_mask = cv2.bilateralFilter(obstacle_mask, 9, 110, 110)
            cv2.imshow("ObstacleMask", obstacle_mask)

            center_mass = fun.detectCenterOfMass(white_mask)
            # cv2.imshow("CenterOfMass", center_mass)

            corners_image = fun.detectCorners(white_mask)
            cv2.imshow("Corners", corners_image)

            contours_image = fun.detectContours(white_mask)
            # im_with_keypoints = detectBlobs(obstacle_mask)
            # Show keypoints
            cv2.imshow("Contours", contours_image)
            # cv2.waitKey(0)

            # for i in range(len(mask)):
            #     for j in range(len(mask)):
            #         if mask[i][j] == 255:
            #             matrix[i][j] = 1
            # grid_matrix = mask/255

            # # fig, ax = plt.subplots()
            # cmap = colors.ListedColormap(['white', 'red'])

            # bounds = [0, 1, 512]
            # norm = colors.BoundaryNorm(bounds, cmap.N)
            # img = plt.imshow(grid_matrix, interpolation='nearest', origin='lower',
            #                  cmap=cmap, norm=norm)
            # plt.show()

            # # plot it
            # ax.imshow(grid_matrix, interpolation='none', cmap=cmap, norm=norm)

            # print(grid_matrix)

            # cv2.cvtColor(output_image, cv2.COLOR_HSV2BGR)
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
