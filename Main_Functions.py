import sim
import cv2
import numpy as np
import matplotlib as mpl
import tcod
import time

SPEED = 4.0
K_GAIN = 3.8
IMG_WIDTH = 512
IMG_HEIGHT = 512
ONE_UNIT_DISTANCE = 6.283185307179586
FORWARD_SPEED = 3.0
PIXELS_PER_METER = 25.6
# GRND_BOT_SIZE = 1


def handleDroneObjects(clientID):
    # --------------------------------- DRONE ---------------------------------
    # Floor orthographic camera for exploration
    res, camera_drone = sim.simxGetObjectHandle(
        clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Body
    res, drone_base_handle = sim.simxGetObjectHandle(
        clientID, 'Quadricopter_base', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle of drone base')

    res, drone_target_handle = sim.simxGetObjectHandle(
        clientID, 'Quadricopter_target', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle of drone target')

    # --------------------------------- GENERAL ---------------------------------
    # Floor
    res, floor = sim.simxGetObjectHandle(
        clientID, 'ResizableFloor_5_25', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Floor')

    return camera_drone, drone_base_handle, drone_target_handle, floor


def handleGroundObjects(clientID):
    # --------------------------------- GROUND ROBOT ---------------------------------
    # Ground robot's perspective vision sensor
    res, camera_gnd = sim.simxGetObjectHandle(
        clientID, 'Ground_vision_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Floor proximity ir sensor
    res, prox_sensor = sim.simxGetObjectHandle(
        clientID, 'proximity_sensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Proximity Sensor')

    # Ground robot body
    res, body_gnd = sim.simxGetObjectHandle(
        clientID, 'Shape', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    # Wheel drive motors
    res, left_motor = sim.simxGetObjectHandle(
        clientID, 'Left_motor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to leftMotor')

    res, right_motor = sim.simxGetObjectHandle(
        clientID, 'Right_motor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to rightMotor')

    # Wheels
    res, leftWheel = sim.simxGetObjectHandle(
        clientID, 'Left_wheel', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to leftWheel')
    res, rightWheel = sim.simxGetObjectHandle(
        clientID, 'Right_wheel', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to rightWheel')

    # Ground robot arm joints
    res, left_joint = sim.simxGetObjectHandle(
        clientID, 'Left_joint', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    res, right_joint = sim.simxGetObjectHandle(
        clientID, 'Right_joint', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    return camera_gnd, prox_sensor, body_gnd, left_motor, right_motor, leftWheel, rightWheel, left_joint, right_joint


def droneInitialMovement(clientID, drone_base_handle, drone_target_handle, floor, drone_viewposition, repeatseed):
    print("Moving to the centre of the scene...")

    drone_base_position = sim.simxGetObjectPosition(
        clientID, drone_base_handle, floor, sim.simx_opmode_blocking)
    # drone_target_position = sim.simxGetObjectPosition(
    #     clientID, drone_target_handle, floor, sim.simx_opmode_blocking)

    # Drone move in z axis
    if(drone_base_position[1][2] <= 8 and repeatseed == 0):
        repeatseed = 1
        for i in range(int(drone_base_position[1][2]+1), 9):
            drone_base_position = sim.simxGetObjectPosition(
                clientID, drone_target_handle, floor, sim.simx_opmode_blocking)
            sim.simxSetObjectPosition(clientID, drone_target_handle, floor, [
                drone_base_position[1][0], drone_base_position[1][1], i], sim.simx_opmode_blocking)
            time.sleep(3)

    # Drone move in x axis
    if(drone_base_position[1][0] != 0 and repeatseed == 1):
        repeatseed = 2
        drone_x_sign = drone_base_position[1][0] / \
            abs(drone_base_position[1][0])
        for i in range(1, ((int(abs(drone_base_position[1][0])))*10)+1):
            drone_base_position = sim.simxGetObjectPosition(
                clientID, drone_target_handle, floor, sim.simx_opmode_blocking)
            sim.simxSetObjectPosition(clientID, drone_target_handle, floor, [
                drone_base_position[1][0] - drone_x_sign*0.1, drone_base_position[1][1], drone_base_position[1][2]], sim.simx_opmode_blocking)
            time.sleep(0.3)
        time.sleep(4)
        drone_base_position = sim.simxGetObjectPosition(
            clientID, drone_target_handle, floor, sim.simx_opmode_blocking)

    if(drone_base_position[1][0] != 0 and repeatseed == 2):
        repeatseed = 3
        drone_y_sign = drone_base_position[1][1] / \
            abs(drone_base_position[1][1])
        for i in range(1, ((int(abs(drone_base_position[1][1])))*10)+1):
            drone_base_position = sim.simxGetObjectPosition(
                clientID, drone_target_handle, floor, sim.simx_opmode_blocking)
            sim.simxSetObjectPosition(clientID, drone_target_handle, floor, [
                drone_base_position[1][0], drone_base_position[1][1] - drone_y_sign*0.1, drone_base_position[1][2]], sim.simx_opmode_blocking)
            time.sleep(0.4)
        time.sleep(4)
        drone_base_position = sim.simxGetObjectPosition(
            clientID, drone_target_handle, floor, sim.simx_opmode_blocking)

    return drone_viewposition, repeatseed, True


# Find the masks by color (obstacles and ground robot)
def findColorsMasks(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Ground robot blue colour
    robot_mask = cv2.inRange(hsv_image,
                             np.array([85, 120, 255]), np.array([95, 255, 255]))
    # cv2.imshow("ROBOT", robot_mask)

    # Green tree colour
    tree_mask = cv2.inRange(hsv_image, np.array(
        [40, 30, 100]), np.array([80, 100, 200]))

    # Detecting ceilings and walls
    white_mask = cv2.inRange(hsv_image, np.array(
        [0, 0, 255]), np.array([0, 0, 255]))

    # Detecting the concrete block
    gray_mask = cv2.inRange(hsv_image, np.array(
        [0, 0, 200]), np.array([2, 0, 202]))
    # cv2.imshow("BLOCK", gray_mask)

    # Detecting ceilings, walls, and concrete blocks
    white_obstacle_mask = cv2.bitwise_or(white_mask, gray_mask)

    return robot_mask, tree_mask, white_obstacle_mask


def findBearMask(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Green tshirt colour
    tshirt_mask = cv2.inRange(hsv_image, np.array(
        [50, 150, 20]), np.array([70, 255, 255]))

    return tshirt_mask


def findMantaMask(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Red colour
    red_mask_1 = cv2.inRange(hsv_image, np.array(
        [0, 120, 20]), np.array([15, 255, 255]))
    red_mask_2 = cv2.inRange(hsv_image, np.array(
        [170, 120, 255]), np.array([180, 255, 255]))
    car_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    return car_mask


def findHospitalMask(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Hospital blue colour
    hospital_mask = cv2.inRange(hsv_image, np.array(
        [110, 120, 20]), np.array([135, 255, 255]))

    return hospital_mask


def detectCenterOfMass(given_image):

    color_image = cv2.cvtColor(given_image, cv2.COLOR_GRAY2BGR)

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


def createFatMap(grid_matrix, fat_number):
    # print("Making walls thicker...")
    map_matrix = np.copy(grid_matrix)

    size_y = len(grid_matrix)
    size_x = len(grid_matrix[0])

    for y in range(size_y):
        for x in range(size_x):
            if (grid_matrix[y][x] == 255):
                cv2.rectangle(map_matrix, (x-fat_number, y-fat_number),
                              (x+fat_number, y+fat_number), 255, -1)

    return map_matrix


def pathToImage(obstacles_image, path):
    path_image = np.copy(obstacles_image)
    path_image = cv2.cvtColor(obstacles_image, cv2.COLOR_GRAY2RGB)

    for point in path:
        cv2.circle(path_image, (point[1], point[0]), 1, (50, 3, 255), -1)

    cv2.circle(path_image, (path[-1][1], path[-1][0]), 5, (255, 3, 214), -1)

    # Save map and path image
    # path_image.dtype = 'uint8'
    # status_path = cv2.imwrite(
    #     'C:/Users/GF63/OneDrive/Escritorio/IR-Summer-Project/Path_maze.jpg', path_image)

    cv2.imshow("Path", path_image)
    return path_image


def aStar(map_matrix, start_y, start_x, end_y, end_x):
    cost_matrix = np.ones((IMG_HEIGHT, IMG_WIDTH), dtype=np.int8)

    for i in range(len(map_matrix)):
        for j in range(len(map_matrix[0])):
            if map_matrix[i][j] == 255:
                cost_matrix[i][j] = 0

    print("Finding AStar path...")

    aStar_graph = tcod.path.AStar(cost_matrix, 1.0)
    try:
        path_list = aStar_graph.get_path(start_y, start_x, end_y, end_x)
        return path_list
    except Exception:
        return []


def getCommands(path):
    print("Getting commands for ground robot...")
    detailed_commands = []
    general_commands = []
    commands_meters = []

    for i in range(len(path) - 1):
        command = [path[i+1][0], path[i+1][1]]
        if path[i][0] == path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Left")
                command.append(90)
            else:
                # print("Right")
                command.append(-90)
                # command.append(270)
        elif path[i][1] == path[i+1][1]:
            if path[i][0] > path[i+1][0]:
                # print("North")
                command.append(0)
            else:
                # print("South")
                command.append(-180)
                # command.append(180)
        elif path[i][0] > path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Upper left")
                command.append(45)
            elif path[i][1] < path[i+1][1]:
                # print("Upper right")
                command.append(-45)
                # command.append(315)
        elif path[i][0] < path[i+1][0]:
            if path[i][1] > path[i+1][1]:
                # print("Lower left")
                command.append(135)
            elif path[i][1] < path[i+1][1]:
                # print("Lower right")
                command.append(-135)
                # command.append(225)
        detailed_commands.append(command)

        # detailed_commands.append(command)

    for i in range(len(detailed_commands)-1):
        if (detailed_commands[i+1][2] != detailed_commands[i][2]):
            general_commands.append(detailed_commands[i])

    # Add the END point to command list
    # general_commands.append(detailed_commands[-1])                                    !!!!!!!!!!!! UNCOMMENT

    commands_meters = pixelsToMeters(general_commands)
    # commands_meters = pixelsToMeters(detailed_commands)

    return commands_meters


def pixelsToMeters(array_pixels):
    print("Converting pixels to Coppelia units...")

    array_meters = np.copy(array_pixels)

    array_meters = [[float(pixel) for pixel in sub_array]
                    for sub_array in array_pixels]

    array_meters = np.array(array_meters)

    array_meters[:, 0] = np.around(
        ((-array_meters[:, 0]/PIXELS_PER_METER)) + 10.0, 2)
    array_meters[:, 1] = np.around(
        ((-array_meters[:, 1]/PIXELS_PER_METER)) + 10.0, 2)

    return array_meters


def armsMovement(clientID, left_joint, right_joint, isRescueDone):
    # Function that returns isReadyToSearch = True when the ground robot has its arms open

    res_left_joint, left_joint_pos = sim.simxGetJointPosition(
        clientID, left_joint, sim.simx_opmode_oneshot)
    res_right_joint, right_joint_pos = sim.simxGetJointPosition(
        clientID, right_joint, sim.simx_opmode_oneshot)

    if (not isRescueDone):
        # Ground robot opens arms before start looking for Mr.York
        if (left_joint_pos < (np.pi-0.15)) and (right_joint_pos < 0.15):
            if (res_left_joint == sim.simx_return_ok) and (res_right_joint == sim.simx_return_ok):
                sim.simxSetJointTargetVelocity(
                    clientID, left_joint, 0.3, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID, right_joint, -0.3, sim.simx_opmode_oneshot)
        else:
            sim.simxSetJointTargetVelocity(
                clientID, left_joint, 0.0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, right_joint, 0.0, sim.simx_opmode_oneshot)
            # Once the ground robot's arms are opened, it is ready to start moving around
            return True, False
    else:
        # The arms start closing to hug Mr. York
        if (res_left_joint == sim.simx_return_ok) and (res_right_joint == sim.simx_return_ok):
            sim.simxSetJointTargetVelocity(
                clientID, left_joint, -0.3, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, right_joint, 0.3, sim.simx_opmode_oneshot)

        left_joint_pos = sim.simxGetJointPosition(
            clientID, left_joint, sim.simx_opmode_oneshot)[1]
        right_joint_pos = sim.simxGetJointPosition(
            clientID, right_joint, sim.simx_opmode_oneshot)[1]
        if (left_joint_pos < -0.05) and (right_joint_pos > 0.078):
            sim.simxSetJointTargetVelocity(
                clientID, left_joint, 0.0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, right_joint, 0.0, sim.simx_opmode_oneshot)
            return False, True

    return False, False


def groundMovement(state, clientID, left_motor, right_motor, motor_speed):
    if state == 'FORWARD':
        sim.simxSetJointTargetVelocity(
            clientID, left_motor, motor_speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, right_motor, motor_speed, sim.simx_opmode_oneshot)
    elif state == 'TURN_LEFT':
        sim.simxSetJointTargetVelocity(
            clientID, left_motor, -motor_speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, right_motor, motor_speed, sim.simx_opmode_oneshot)
    elif state == 'TURN_RIGHT':
        sim.simxSetJointTargetVelocity(
            clientID, left_motor, motor_speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, right_motor, -motor_speed, sim.simx_opmode_oneshot)
    elif state == 'STOP':
        sim.simxSetJointTargetVelocity(
            clientID, left_motor, 0.0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, right_motor, 0.0, sim.simx_opmode_oneshot)


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


def changeRadiansToDegrees(angle):
    return np.rad2deg(angle)


def checkAngle(current_angle, desired_angle):
    return (abs(desired_angle-current_angle) < 1.0)


def checkPosition(current_y, current_x, desired_y, desired_x):
    xposition = 0
    yposition = 0
    if (abs(current_x - desired_x) < 0.3):
        xposition = 1
    if (abs(current_y - desired_y) < 0.3):
        yposition = 1
    return [xposition, yposition]


def deltaSpeed(delta):
    if delta > 6.0:
        delta = -6.0
    elif delta < -6.0:
        delta = 6.0
    return delta
