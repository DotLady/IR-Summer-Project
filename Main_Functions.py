import sim
import cv2
import numpy as np
import matplotlib as mpl
import tcod

SPEED = 5.0
K_GAIN = 1.2
IMG_WIDTH = 512
IMG_HEIGHT = 512
PIXELS_PER_METER = 25.6
# GRND_BOT_SIZE = 1

# Find the masks by color (red, green and blue)


def handleObjects(clientID_drone, clientID_gnd):
    # --------------------------------- DRONE ---------------------------------
    # Floor orthographic camera for exploration
    res, camera_drone = sim.simxGetObjectHandle(
        clientID_drone, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Body
    res, body_drone = sim.simxGetObjectHandle(
        clientID_drone, 'Quadricopter_target', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    # --------------------------------- GROUND ROBOT ---------------------------------
    # Ground robot's perspective vision sensor
    res, camera_gnd = sim.simxGetObjectHandle(
        clientID_gnd, 'ground_vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Floor proximity ir sensor
    res, ir_sensor = sim.simxGetObjectHandle(
        clientID_gnd, 'ground_IR_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Proximity Sensor')

    # Ground robot body
    res, body_gnd = sim.simxGetObjectHandle(
        clientID_gnd, 'Shape', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    # Wheel drive motors
    res, leftMotor = sim.simxGetObjectHandle(
        clientID_gnd, 'leftMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to leftMotor')
    res, rightMotor = sim.simxGetObjectHandle(
        clientID_gnd, 'rightMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to rightMotor')

    # --------------------------------- GENERAL ---------------------------------
    # Floor
    res, floor = sim.simxGetObjectHandle(
        clientID_gnd, 'ResizableFloor_5_25', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Floor')

    return camera_drone, body_drone, camera_gnd, ir_sensor, body_gnd, leftMotor, rightMotor, floor


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


def findBearMask(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Green tshirt colour
    greenMin_tshirt = np.array([40, 120, 20])
    greenMax_tshirt = np.array([80, 255, 255])
    tshirt_mask = cv2.inRange(hsv_image, greenMin_tshirt, greenMax_tshirt)

    return tshirt_mask


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
    general_commands.append(detailed_commands[-1])

    commands_meters = pixelsToMeters(general_commands)

    return commands_meters


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


def eulerAnglesToDegrees(euler_angle):
    angle_degrees = (euler_angle*360.0)/(2*np.pi)

    new_angle = angle_degrees

    if angle_degrees == 360.0:
        new_angle = 0.0
    elif angle_degrees < 0.0:
        new_angle += 360

    return new_angle


def groundMovement(state, clientID, leftMotor, rightMotor, delta):
    if state == 'FORWARD':
        sim.simxSetJointTargetVelocity(
            clientID, leftMotor, SPEED + delta, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, rightMotor, SPEED - delta, sim.simx_opmode_oneshot)
    elif state == 'TURN_LEFT':
        sim.simxSetJointTargetVelocity(
            clientID, leftMotor, -SPEED, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, rightMotor, SPEED, sim.simx_opmode_oneshot)
    elif state == 'TURN_RIGHT':
        sim.simxSetJointTargetVelocity(
            clientID, leftMotor, SPEED, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, rightMotor, -SPEED, sim.simx_opmode_oneshot)
    elif state == 'STOP':
        sim.simxSetJointTargetVelocity(
            clientID, leftMotor, 0.0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, rightMotor, 0.0, sim.simx_opmode_oneshot)


# Function to find the error between the actual center of the image
# and the centre of mass
def mapCenter(central_X, in_min, in_max, out_min, out_max):
    return (central_X - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# The difference between the centers is considered through the
# variable delta, which is added and substracted to the SPEED
# of the robot's wheels
def controllerMove(central_X):
    error = mapCenter(central_X, 0.0, IMG_WIDTH, -1.0, 1.0)
    delta = error*K_GAIN

    return delta
