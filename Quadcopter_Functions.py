'''File that includes all the functions used in the Quadcopter.py file'''

import sim
import cv2
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
from matplotlib import colors
import tcod

IMG_WIDTH = 512
IMG_HEIGHT = 512
# GRND_BOT_SIZE = 1

# Find the masks by color (red, green and blue)


def findColorsMasks(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Red colour
    redMin_1 = np.array([0, 120, 20])
    redMax_1 = np.array([15, 255, 255])
    redMin_2 = np.array([170, 120, 20])
    redMax_2 = np.array([180, 255, 255])
    red_mask_1 = cv2.inRange(hsv_image, redMin_1, redMax_1)
    red_mask_2 = cv2.inRange(hsv_image, redMin_2, redMax_2)
    car_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    # Hospital blue colour
    blueMin = np.array([85, 120, 20])
    blueMax = np.array([135, 255, 255])
    hospital_mask = cv2.inRange(hsv_image, blueMin, blueMax)

    # Ground robot blue colour
    blueMin = np.array([0, 255, 255])
    blueMax = np.array([0, 255, 255])
    robot_mask = cv2.inRange(hsv_image, blueMin, blueMax)

    # Green tree colour
    greenMin_tree = np.array([40, 0, 20])
    greenMax_tree = np.array([80, 120, 255])
    tree_mask = cv2.inRange(hsv_image, greenMin_tree, greenMax_tree)

    # Detecting ceilings and walls
    lower_white = np.array([0, 0, 252])
    upper_white = np.array([0, 0, 255])
    white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

    # Detecting the concrete block
    lower_gray = np.array([0, 0, 200])
    upper_gray = np.array([2, 0, 202])
    gray_mask = cv2.inRange(hsv_image, lower_gray, upper_gray)

    # Detecting ceilings, walls, and concrete blocks
    white_obstacle_mask = cv2.bitwise_or(white_mask, gray_mask)

    return hospital_mask, car_mask, tree_mask, white_obstacle_mask


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


def detectCenterOfMass(given_image, blurImage):
    if blurImage:
        given_image = cv2.medianBlur(given_image, 21)

    color_image = cv2.cvtColor(given_image, cv2.COLOR_GRAY2BGR)

    # temp_image = regionOfInterest(given_image, [0.05, 0.05, 0.95, 0.95])

    ret, thresh = cv2.threshold(given_image, 0, 255, 0)
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cX = IMG_WIDTH/2
    cY = IMG_HEIGHT/2

    for c in contours:
        # compute the center of the contour
        M = cv2.moments(c)
        if (M["m10"] and M["m01"] and M["m00"]) != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.drawContours(color_image, [c], -1, (0, 255, 0), 2)
            cv2.circle(color_image, (cX, cY), 7, (0, 255, 0), -1)

    return color_image, cX, cY


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


def createFatMap(grid_matrix):
    print("Making walls thicker...")
    map_matrix = np.copy(grid_matrix)

    size_y = len(grid_matrix)
    size_x = len(grid_matrix[0])

    for y in range(size_y):
        for x in range(size_x):
            if (grid_matrix[y][x] == 255):
                cv2.circle(map_matrix, (x, y), 10, 255, -1)

    return map_matrix


def pathToImage(obstacles_image, path):
    path_image = np.copy(obstacles_image)
    path_image = cv2.cvtColor(obstacles_image, cv2.COLOR_GRAY2RGB)

    for point in path:
        # path_image[point[0]][point[1]] = 255
        cv2.circle(path_image, (point[1], point[0]), 1, (50, 3, 255), -1)

    cv2.circle(path_image, (path[0][1], path[0][0]), 6, (104, 255, 3), -1)
    cv2.circle(path_image, (path[-1][1], path[-1][0]), 6, (255, 3, 214), -1)

    return path_image


def mapToCostMatrix(map_matrix):
    cost_matrix = np.ones((IMG_HEIGHT, IMG_WIDTH), dtype=np.int8)

    for i in range(len(map_matrix)):
        for j in range(len(map_matrix[0])):
            if map_matrix[i][j] == 255:
                cost_matrix[i][j] = 0

    return cost_matrix


def pathFinder(map_matrix, start_y, start_x, end_y, end_x):
    print("Finding path...")
    cost_matrix = mapToCostMatrix(map_matrix)

    graph = tcod.path.SimpleGraph(cost=cost_matrix, cardinal=1, diagonal=3)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root((start_y, start_x))
    path_list = pf.path_to((end_y, end_x)).tolist()

    return path_list


def aStar(map_matrix, start_y, start_x, end_y, end_x):
    print("Finding path...")
    cost_matrix = mapToCostMatrix(map_matrix)

    aStar_graph = tcod.path.AStar(cost_matrix, 0)
    path_list = aStar_graph.get_path(start_y, start_x, end_y, end_x)

    return path_list


def getCommands(path):
    print("Getting commands for ground robot...")
    detailed_commands = []
    general_commands = []

    for i in range(len(path) - 1):
        command = [path[i+1][0], path[i+1][1]]
        if path[i][0] == path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Izquierda")
                command.append(270)
            else:
                # print("Derecha")
                command.append(90)
        elif path[i][1] == path[i+1][1]:
            if path[i][0] > path[i+1][0]:
                # print("Norte")
                command.append(180)
            else:
                # print("Sur")
                command.append(0)
        elif path[i][0] > path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Diagonal superior izquierda")
                command.append(225)
            elif path[i][1] < path[i+1][1]:
                # print("Diagonal superior derecha")
                command.append(135)
        elif path[i][0] < path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Diagonal inferior izquierda")
                command.append(315)
            elif path[i][1] < path[i+1][1]:
                # print("Diagonal inferior derecha")
                command.append(45)
        detailed_commands.append(command)

    for i in range(len(detailed_commands)-1):
        if (detailed_commands[i+1][2] != detailed_commands[i][2]):
            general_commands.append(detailed_commands[i])

    # Add the END point to command list
    general_commands.append(detailed_commands[-1])

    return general_commands


def pixelsToMeters(commands_list):
    commands_meters = np.copy(commands_list)
    commands_meters = [[float(x) for x in sublist]
                       for sublist in commands_meters]

    for i in range(len(commands_list)):
        # Y coordinates (vertical)
        if commands_list[i][0] == 0:
            commands_meters[i][0] = 10
        elif commands_list[i][0] > 0:
            if commands_list[i][0] < 256:
                commands_meters[i][0] = commands_list[i][0]*10/512
            elif commands_list[i][0] == 256:
                commands_meters[i][0] = 0
            elif commands_list[i][0] == 512:
                commands_meters[i][0] = -10
            else:
                commands_meters[i][0] = -(commands_list[i][0]*10/512)

        # X coordinates (horizontal)
        if commands_list[i][1] == 0:
            commands_meters[i][1] = -10
        elif commands_list[i][1] > 0:
            if commands_list[i][1] < 256:
                commands_meters[i][1] = -10 + (commands_list[i][1]*10/512)
            elif commands_list[i][1] == 256:
                commands_meters[i][1] = 0
            elif commands_list[i][1] == 512:
                commands_meters[i][1] = 10
            else:
                commands_meters[i][1] = commands_list[i][1]*10/512

    commands_meters = [[round(x, 4) for x in sublist]
                       for sublist in commands_meters]

    return commands_meters
