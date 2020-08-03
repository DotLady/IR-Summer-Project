import sim
import cv2
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
from matplotlib import colors
import heapq

IMG_WIDTH = 512
IMG_HEIGHT = 512
# GRND_BOT_SIZE = 1

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
    car_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    # Blue colour
    blueMin = np.array([85, 120, 20])
    blueMax = np.array([135, 255, 255])
    hospital_mask = cv2.inRange(hsv_image, blueMin, blueMax)

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

    return hospital_mask, car_mask, tree_mask, white_mask


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
    # blur_image = cv2.medianBlur(given_image, 21)
    color_image = cv2.cvtColor(given_image, cv2.COLOR_GRAY2BGR)

    # temp_image = regionOfInterest(given_image, [0.05, 0.05, 0.95, 0.95])

    ret, thresh = cv2.threshold(given_image, 0, 255, 0)
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        # compute the center of the contour
        M = cv2.moments(c)
        if (M["m10"] and M["m01"] and M["m00"]) != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            # cv2.drawContours(color_image, [c], -1, (0, 255, 0), 2)
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


def createMap(grid_matrix):
    map_matrix = np.copy(grid_matrix)

    size_y = len(grid_matrix)
    size_x = len(grid_matrix[0])

    for y in range(size_y):
        for x in range(size_x):
            if (grid_matrix[y][x] == 1):
                if (x > 0):
                    map_matrix[y][x-1] = 1
                if (x < size_x-1):
                    map_matrix[y][x+1] = 1
                if (y > 0):
                    map_matrix[y-1][x] = 1
                if (y < size_y-1):
                    map_matrix[y+1][x] = 1

    return map_matrix

# Node class for the A* algorithm


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.start_distance = 0
        self.end_distance = 0
        self.total_score = 0

    def __eq__(self, other):
        return self.position == other.position


def return_path(current_node, maze):
    path = []
    num_rows, num_cols = np.shape(maze)
    result = [[-1 for i in range(num_cols)] for j in range(num_rows)]
    current = current_node

    while current is not None:
        path.append(current.position)
        print(current.position)
        current = current.parent
    path = path[::-1]  # Return reversed path

    for i in range(len(path) - 1):
        if (path[i+1][0] == path[i][0]):
            if (path[i+1][1] > path[i][1]):
                print("Move East towards position ", path[i+1])
                print("(", path[i+1][0], ",", path[i+1][1], ", 90 )")
            else:
                print("Move West towards position ", path[i+1])
                print("(", path[i+1][0], ",", path[i+1][1], ", 270 )")
        elif (path[i+1][0] < path[i][0]):
            print("Move North towards position ", path[i+1])
            print("(", path[i+1][0], ",", path[i+1][1], ", 180 )")
        else:
            print("Move South towards position ", path[i+1])
            print("(", path[i+1][0], ",", path[i+1][1], ", 0 )")

    start_value = 0
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return result


def searchPath(maze, cost, start, end):
    start_node = Node(None, tuple(start))
    start_node.start_distance = start_node.end_distance = start_node.total_score = 0
    end_node = Node(None, tuple(end))
    end_node.start_distance = end_node.end_distance = end_node.total_score = 0

    # Initialize both open and closed list
    yet_to_visit_list = []
    visited_list = []
    yet_to_visit_list.append(start_node)

    outer_iterations = 0
    max_iterations = (len(np.floor_divide(maze, 2))) ** 10

    move = [[-1, 0], [0, -1], [1, 0], [0, 1]]

    num_rows, num_cols = np.shape(maze)

    while len(yet_to_visit_list) > 0:
        outer_iterations += 1
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.total_score < current_node.total_score:
                current_node = item
                current_index = index

        if outer_iterations > max_iterations:
            print("Giving up on pathfinding too many iterations")
            return return_path(current_node, maze)

        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        if current_node == end_node:
            return return_path(current_node, maze)

        children = []

        for new_position in move:  # Adjacent squares

            # Get node position
            node_position = (
                current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (maze boundaries)
            if node_position[0] > (num_rows - 1) or node_position[0] < 0 or node_position[1] > (num_cols - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # Create the f, start_distance, and h values
            child.start_distance = current_node.start_distance + cost
            child.end_distance = ((child.position[0] - end_node.position[0]) ** 2) + (
                (child.position[1] - end_node.position[1]) ** 2)
            child.total_score = child.start_distance + child.end_distance

            # Child is already in the open list
            if len([i for i in yet_to_visit_list if child == i and child.start_distance > i.start_distance]) > 0:
                continue

            # Add the child to the open list
            yet_to_visit_list.append(child)
