import sim
import cv2
import numpy as np
import Main_Functions as fun
import tcod
import time


IMG_WIDTH = 512
IMG_HEIGHT = 512
ONE_UNIT_DISTANCE = 6.283185307179586
FORWARD_SPEED = 12.0
TURNING_SPEED = 5.0
APPROACHING_BEAR_SPEED = 4.0
PIXELS_PER_METER = 25.6
THICK_NUMBER = 16


def GETTING_MAP(original, text):
    # cv2.imshow("Drone camera", original)
    print("Creating map...")

    aStar_path = []
    commands = []

    # Find masks for hospital, car and obstacles
    robot_mask, tree_mask, white_obstacle_mask = fun.findColorsMasks(
        original)
    manta_mask = fun.findMantaMask(original)

    obstacles_image = cv2.bitwise_or(tree_mask, white_obstacle_mask)

    thick_mask = fun.createFatMap(white_obstacle_mask, THICK_NUMBER)
    thick_tree = fun.createFatMap(tree_mask, 8)
    map_matrix = cv2.bitwise_or(thick_tree, thick_mask)

    # Find START and END coordinates
    start_x, start_y = fun.detectCenterOfMass(robot_mask)
    # print("Ground robot centre: (", start_x, ", ", start_y, ")")

    if (text == 'MAP_TO_RESCUE'):
        temp_x, temp_y = fun.detectCenterOfMass(manta_mask)
        end_x = temp_x  # - 15
        end_y = temp_y + 19
        # print("Red manta's centre: (", end_x, ", ", end_y, ")")
    else:
        hospital_mask = fun.findHospitalMask(original)
        end_x, end_y = fun.detectCenterOfMass(hospital_mask)
        # print("Hospital's centre: (", end_x, ", ", end_y, ")")
        thick_manta = fun.createFatMap(manta_mask, 1)
        map_matrix = cv2.bitwise_or(map_matrix, thick_manta)

    # Path Finding algorithm
    aStar_path = fun.aStar(map_matrix, start_y, start_x, end_y, end_x)
    # aStar_path = fun.pathFinder(map_matrix, start_y, start_x, end_y, end_x)

    if (len(aStar_path) != 0):
        path_image = fun.pathToImage(obstacles_image, aStar_path)
        commands = fun.getCommands(aStar_path)
        return commands, True
    else:
        return [], False


# def MOVING_TO_PATH(commands, clientID_gnd, left_motor, right_motor):


def SEARCHING_BEAR(color_values, tshirt_x, clientID, left_motor, right_motor, prox_sensor):
    # Function returns isRescueDone, isReadyToSearch
    # Calculated offset
    delta = fun.controllerMove(tshirt_x)

    # Look up for Mr York and move towards him
    if (len(color_values) == 1):
        if (color_values[0] == 0):
            fun.groundMovement(
                'TURN_RIGHT', clientID, left_motor, right_motor, TURNING_SPEED)

        elif (color_values[0] == 255):
            fun.groundMovement(
                'STOP', clientID, left_motor, right_motor, 0.0)
            return True, False

    else:
        sim.simxSetJointTargetVelocity(
            clientID, left_motor, APPROACHING_BEAR_SPEED + delta, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, right_motor, APPROACHING_BEAR_SPEED - delta, sim.simx_opmode_oneshot)
        isDetecting = sim.simxReadProximitySensor(
            clientID, prox_sensor, sim.simx_opmode_oneshot)[1]
        if isDetecting:
            fun.groundMovement(
                'STOP', clientID, left_motor, right_motor, 0.0)
            return True, False

    return False, True
