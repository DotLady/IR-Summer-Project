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
import time

IMG_WIDTH = 512
IMG_HEIGHT = 512

grid_matrix = np.zeros((512, 512))
drone_viewposition = [0,0,8]
repeatseed = 0
# -------------------------------------- START PROGRAM --------------------------------------
# Start Program and just in case, close all opened connections

# Connect to simulator running on localhost
# V-REP runs on port 19997, a script opens the API on port 19999
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 3)

# Connect to the simulation
if clientID != -1:
    print('Connected to remote API server')

    # Get handles to simulation objects
    print('Obtaining handles of simulation objects')
    res,leftMotor = sim.simxGetObjectHandle(clientID, 'leftMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok: print('Could not get handle to leftMotor')
    res,rightMotor = sim.simxGetObjectHandle(clientID, 'rightMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok: print('Could not get handle to rightMotor')
    

    
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

# -------------------------------------- Main Control Loop --------------------------------------

    # Start main control loop
    print('Starting control loop')

    res, resolution, image_pers = sim.simxGetVisionSensorImage(
        clientID, camera_pers, 0, sim.simx_opmode_streaming)
    res, resolution, image_orth = sim.simxGetVisionSensorImage(
        clientID, camera_orth, 0, sim.simx_opmode_streaming)


    while (sim.simxGetConnectionId(clientID) != -1):
        
        sim.simxSetJointTargetVelocity(clientID, leftMotor, 1.0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, rightMotor, 1.0, sim.simx_opmode_oneshot)
        # Get image from CameraAa
        res_pers, resolution, image_pers = sim.simxGetVisionSensorImage(
            clientID, camera_pers, 0, sim.simx_opmode_buffer)
        res, resolution, image_orth = sim.simxGetVisionSensorImage(
            clientID, camera_orth, 0, sim.simx_opmode_buffer)

        #get drone position
        drone_base_position =  sim.simxGetObjectPosition(clientID, drone_base_hanlde, floor, sim.simx_opmode_blocking)
        drone_target_position = sim.simxGetObjectPosition(clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        print(drone_base_position)
        
        #Drone move in z axis
        if(drone_base_position[1][2] <= 8 and repeatseed == 0 ):
            repeatseed = 1
            for i in range(int(drone_base_position[1][2]+1),9):
                drone_base_position = sim.simxGetObjectPosition(clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
                sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [drone_base_position[1][0],drone_base_position[1][1],i], sim.simx_opmode_blocking)
                print(drone_base_position)
                time.sleep(2)
        #Drone move in x axis
        if(drone_base_position[1][0] !=0 and repeatseed == 1):
            repeatseed = 2
            drone_x_sign = drone_base_position[1][0] / abs(drone_base_position[1][0])
            for i in range(1,((int(abs(drone_base_position[1][0])))*10)+1):
                drone_base_position = sim.simxGetObjectPosition(clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
                sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [drone_base_position[1][0] - drone_x_sign*0.1
                                                                                 ,drone_base_position[1][1],drone_base_position[1][2]], sim.simx_opmode_blocking)
                print(drone_base_position)
                time.sleep(0.1)
            time.sleep(4)
            drone_base_position = sim.simxGetObjectPosition(clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
            print(drone_base_position)
        
        if(drone_base_position[1][0] !=0 and repeatseed == 2):
            repeatseed = 3
            drone_y_sign = drone_base_position[1][1] / abs(drone_base_position[1][1])
            for i in range(1,((int(abs(drone_base_position[1][1])))*10)+1):
                drone_base_position = sim.simxGetObjectPosition(clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
                sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [drone_base_position[1][0]
                                                                                 ,drone_base_position[1][1] - drone_y_sign*0.1, drone_base_position[1][2]], sim.simx_opmode_blocking)
                print(drone_base_position)
                time.sleep(0.1)
            time.sleep(4)
            drone_base_position = sim.simxGetObjectPosition(clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
            print(drone_base_position)
        #image process
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
        
        #drone move to center

        
else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
