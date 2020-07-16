#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: Daniela Ordóñez Tapiaf}

"""

import sim
import cv2
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
from matplotlib import colors

IMG_WIDTH = 512
IMG_HEIGHT = 512

grid_matrix = np.zeros((512, 512))


# Find the masks by color (red, green and blue)
def findColorsMasks(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Red colour
    redMin_1 = np.array([0, 120, 20])
    redMax_1 = np.array([10, 255, 255])
    redMin_2 = np.array([170, 120, 20])
    redMax_2 = np.array([180, 255, 255])
    red_mask_1 = cv2.inRange(hsv_image, redMin_1, redMax_1)
    red_mask_2 = cv2.inRange(hsv_image, redMin_2, redMax_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    # Blue colour
    blueMin = np.array([85, 120, 20])
    blueMax = np.array([135, 255, 255])
    blue_mask = cv2.inRange(hsv_image, blueMin, blueMax)

    # Green tshirt colour
    greenMin_tshirt = np.array([40, 120, 20])
    greenMax_tshirt = np.array([80, 255, 255])
    tshirt_mask = cv2.inRange(hsv_image, greenMin_tshirt, greenMax_tshirt)

    # Green tree colour
    greenMin_tree = np.array([40, 0, 20])
    greenMax_tree = np.array([80, 120, 255])
    tree_mask = cv2.inRange(hsv_image, greenMin_tree, greenMax_tree)

    lower_white = np.array([0, 0, 0])
    upper_white = np.array([0, 0, 255])
    white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

    map_mask = cv2.bitwise_or(blue_mask, red_mask)

    return map_mask, tree_mask, white_mask


def regionOfInterest(given_image, roi_ratios):
    x_ratio_start = roi_ratios[0]
    y_ratio_start = roi_ratios[1]
    x_ratio_end = roi_ratios[2]
    y_ratio_end = roi_ratios[3]
    mask = np.zeros_like(given_image)

    # -------------------------------------------------------------------------
    # | (x_ratio_start, y_ratio_start)           (x_ratio_end, y_ratio_start) |
    # |                                                                       |
    # | (x_ratio_start, y_ratio_end)             (x_ratio_end, y_ratio_end)   |
    # -------------------------------------------------------------------------

    vertices = np.array([[IMG_WIDTH*x_ratio_start, IMG_HEIGHT*y_ratio_start], [IMG_WIDTH*x_ratio_start, IMG_HEIGHT*y_ratio_end],
                         [IMG_WIDTH*x_ratio_end, IMG_HEIGHT*y_ratio_end], [IMG_WIDTH*x_ratio_end, IMG_HEIGHT*y_ratio_start]], np.int32)
    cv2.fillPoly(mask, [vertices], (255, 255, 255))

    return_image = cv2.bitwise_and(given_image, mask)
    return return_image


def detectCenterOfMass(given_image):
    blur_image = cv2.medianBlur(given_image, 21)
    color_image = cv2.cvtColor(blur_image, cv2.COLOR_GRAY2BGR)

    temp_image = regionOfInterest(blur_image, [0.05, 0.05, 0.95, 0.95])

    ret, thresh = cv2.threshold(temp_image, 0, 255, 0)
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        # compute the center of the contour
        M = cv2.moments(c)
        if (M["m10"] and M["m01"] and M["m00"]) != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.drawContours(color_image, [c], -1, (0, 255, 0), 2)
            cv2.circle(color_image, (cX, cY), 7, (0, 0, 255), -1)

    return color_image


def detectCorners(given_image):
    # color_image is in BGR
    blur_image = cv2.medianBlur(given_image, 21)
    color_image = cv2.cvtColor(blur_image, cv2.COLOR_GRAY2BGR)

    temp_image = regionOfInterest(blur_image, [0.05, 0.05, 0.95, 0.95])

    corners = cv2.goodFeaturesToTrack(temp_image, 10, 0.01, 1)

    if corners is not None:
        for corner in corners:
            # Get the coordinates of the found corners
            x, y = corner.ravel()
            cv2.circle(color_image, (int(x), int(y)), 5, (0, 255, 0), -1)

    return color_image


def detectContours(given_image):
    # color_image is in BGR
    blur_image = cv2.medianBlur(given_image, 21)
    color_image = cv2.cvtColor(blur_image, cv2.COLOR_GRAY2BGR)

    temp_image = regionOfInterest(blur_image, [0.05, 0.05, 0.95, 0.95])
    # obtener los contornos
    contours, _ = cv2.findContours(
        temp_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # dibujar los contornos
    cv2.drawContours(color_image, contours, -1, (0, 0, 255), 2, cv2.LINE_AA)

    return color_image


def detectBlobs(given_image):
    # given_image = cv2.cvtColor(given_image, cv2.COLOR_BGR2GRAY)
    # Set up the detector with default parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 100
    params.maxThreshold = 400

    # Filter by Color.
    params.filterByColor = True
    params.blobColor = 255

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 5

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs
    keypoints = detector.detect(given_image)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(given_image, keypoints, np.array(
        []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return im_with_keypoints


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

            map_mask, tree_mask, white_mask = findColorsMasks(original)
            obstacle_mask = cv2.bitwise_or(tree_mask, white_mask)

            output_image = cv2.bitwise_and(
                original, original, mask=map_mask)
            # cv2.imshow("MapMask", output_image)

            obstacle_mask = cv2.bilateralFilter(obstacle_mask, 9, 110, 110)
            cv2.imshow("ObstacleMask", obstacle_mask)

            center_mass = detectCenterOfMass(white_mask)
            # cv2.imshow("CenterOfMass", center_mass)

            corners_image = detectCorners(white_mask)
            cv2.imshow("Corners", corners_image)

            contours_image = detectContours(white_mask)
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
