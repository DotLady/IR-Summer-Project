'''File that includes all the functions used in the Quadcopter.py file'''

import sim
import cv2
import numpy as np
import tcod
import time

IMG_WIDTH = 512
IMG_HEIGHT = 512
PIXELS_PER_METER = 25.6
# GRND_BOT_SIZE = 1


def droneInitialMovement(clientID, drone_base_hanlde, drone_target_hanlde, floor):
    drone_base_position = sim.simxGetObjectPosition(
        clientID, drone_base_hanlde, floor, sim.simx_opmode_blocking)
    drone_target_position = sim.simxGetObjectPosition(
        clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
    print(drone_base_position)

    # Drone move in z axis
    if(drone_base_position[1][2] <= 8 and repeatseed == 0):
        repeatseed = 1
        for i in range(int(drone_base_position[1][2]+1), 9):
            drone_base_position = sim.simxGetObjectPosition(
                clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
            sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [
                drone_base_position[1][0], drone_base_position[1][1], i], sim.simx_opmode_blocking)
            print(drone_base_position)
            time.sleep(2)

    # Drone move in x axis
    if(drone_base_position[1][0] != 0 and repeatseed == 1):
        repeatseed = 2
        drone_x_sign = drone_base_position[1][0] / \
            abs(drone_base_position[1][0])
        for i in range(1, ((int(abs(drone_base_position[1][0])))*10)+1):
            drone_base_position = sim.simxGetObjectPosition(
                clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
            sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [
                drone_base_position[1][0] - drone_x_sign*0.1, drone_base_position[1][1], drone_base_position[1][2]], sim.simx_opmode_blocking)
            print(drone_base_position)
            time.sleep(0.1)
        time.sleep(4)
        drone_base_position = sim.simxGetObjectPosition(
            clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        print(drone_base_position)

    if(drone_base_position[1][0] != 0 and repeatseed == 2):
        repeatseed = 3
        drone_y_sign = drone_base_position[1][1] / \
            abs(drone_base_position[1][1])
        for i in range(1, ((int(abs(drone_base_position[1][1])))*10)+1):
            drone_base_position = sim.simxGetObjectPosition(
                clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
            sim.simxSetObjectPosition(clientID, drone_target_hanlde, floor, [
                drone_base_position[1][0], drone_base_position[1][1] - drone_y_sign*0.1, drone_base_position[1][2]], sim.simx_opmode_blocking)
            print(drone_base_position)
            time.sleep(0.1)
        time.sleep(4)
        drone_base_position = sim.simxGetObjectPosition(
            clientID, drone_target_hanlde, floor, sim.simx_opmode_blocking)
        print(drone_base_position)


# Find the masks by color (red, green and blue)
def findColorsMasks(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Red colour
    redMin_1 = np.array([0, 120, 20])
    redMax_1 = np.array([15, 255, 255])
    redMin_2 = np.array([170, 120, 255])
    redMax_2 = np.array([180, 255, 255])
    red_mask_1 = cv2.inRange(hsv_image, redMin_1, redMax_1)
    red_mask_2 = cv2.inRange(hsv_image, redMin_2, redMax_2)
    car_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    # Hospital blue colour
    # blueMin = np.array([85, 120, 20])
    # blueMax = np.array([135, 255, 255])
    # hospital_mask = cv2.inRange(hsv_image, blueMin, blueMax)

    # Ground robot blue colour
    blueMin = np.array([85, 120, 255])
    blueMax = np.array([95, 255, 255])
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

    return robot_mask, car_mask, tree_mask, white_obstacle_mask


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
            cv2.circle(color_image, (cX, cY), 5, (0, 255, 0), -1)

    # cv2.imshow("Center of mass", color_image)

    return cX, cY


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
                cv2.circle(map_matrix, (x, y), 13, 255, -1)

    return map_matrix


def pathToImage(obstacles_image, path):
    path_image = np.copy(obstacles_image)
    path_image = cv2.cvtColor(obstacles_image, cv2.COLOR_GRAY2RGB)

    for point in path:
        cv2.circle(path_image, (point[1], point[0]), 1, (50, 3, 255), -1)

    cv2.circle(path_image, (path[0][1], path[0][0]), 6, (104, 255, 3), -1)
    cv2.circle(path_image, (path[-1][1], path[-1][0]), 6, (255, 3, 214), -1)

    return path_image


def pathFinder(map_matrix, start_y, start_x, end_y, end_x):
    cost_matrix = np.ones((IMG_HEIGHT, IMG_WIDTH), dtype=np.int8)

    for i in range(len(map_matrix)):
        for j in range(len(map_matrix[0])):
            if map_matrix[i][j] == 255:
                cost_matrix[i][j] = 0

    print("Finding path...")

    graph = tcod.path.SimpleGraph(cost=cost_matrix, cardinal=1, diagonal=3)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root((start_y, start_x))
    path_list = pf.path_to((end_y, end_x)).tolist()

    return path_list


def aStar(map_matrix, start_y, start_x, end_y, end_x):
    cost_matrix = np.ones((IMG_HEIGHT, IMG_WIDTH), dtype=np.int8)

    for i in range(len(map_matrix)):
        for j in range(len(map_matrix[0])):
            if map_matrix[i][j] == 255:
                cost_matrix[i][j] = 0

    print("Finding path...")

    aStar_graph = tcod.path.AStar(cost_matrix, 0)
    path_list = aStar_graph.get_path(start_y, start_x, end_y, end_x)

    return path_list


def getCommands(path):
    print("Getting commands for ground robot...")
    detailed_commands = []
    general_commands = []
    commands_meters = []

    for i in range(len(path) - 1):
        command = [path[i+1][0], path[i+1][1]]
        if path[i][0] == path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Izquierda")
                command.append(90)
            else:
                # print("Derecha")
                command.append(270)
        elif path[i][1] == path[i+1][1]:
            if path[i][0] > path[i+1][0]:
                # print("Norte")
                command.append(0)
            else:
                # print("Sur")
                command.append(180)
        elif path[i][0] > path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Diagonal superior izquierda")
                command.append(45)
            elif path[i][1] < path[i+1][1]:
                # print("Diagonal superior derecha")
                command.append(315)
        elif path[i][0] < path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Diagonal inferior izquierda")
                command.append(135)
            elif path[i][1] < path[i+1][1]:
                # print("Diagonal inferior derecha")
                command.append(225)
        detailed_commands.append(command)

    for i in range(len(detailed_commands)-1):
        if (detailed_commands[i+1][2] != detailed_commands[i][2]):
            general_commands.append(detailed_commands[i])

    # Add the END point to command list
    print("Dets: ", len(detailed_commands))
    # general_commands.append(detailed_commands[-1])

    commands_meters = pixelsToMeters(general_commands)

    if(len(commands_meters) != 0):
        return commands_meters, True
    else:
        return commands_meters, False


def pixelsToMeters(array_pixels):
    print("Converting pixels to Coppelia units...")

    array_meters = np.copy(array_pixels)

    array_meters = [[float(pixel) for pixel in sub_array]
                    for sub_array in array_pixels]

    array_meters = np.array(array_meters)

    array_meters[:, 0] = np.around(
        ((-array_meters[:, 0]/PIXELS_PER_METER)) + 10.0, 4)
    array_meters[:, 1] = np.around(
        ((-array_meters[:, 1]/PIXELS_PER_METER)) + 10.0, 4)

    return array_meters
