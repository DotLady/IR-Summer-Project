import sim
import cv2
import numpy as np
import Main_Functions as fun
import tcod
import time


IMG_WIDTH = 512
IMG_HEIGHT = 512
TURNING_SPEED = 3.0
APPROACHING_BEAR_SPEED = 2.0
THICK_NUMBER = 18


def GETTING_MAP(original, text):
    print("Creating map...")

    aStar_path = []
    commands = []

    # Find masks for ground robot and obstacles
    robot_mask, tree_mask, white_obstacle_mask = fun.findColorsMasks(
        original)
    # Find mask for red manta
    manta_mask = fun.findMantaMask(original)

    # Combine into one image all the obstacles (walls, ceilings and trees)
    obstacles_image = cv2.bitwise_or(tree_mask, white_obstacle_mask)

    # Make the obstacles thicker to take into account the size of the ground robot.
    thick_mask = fun.createFatMap(white_obstacle_mask, THICK_NUMBER)
    thick_tree = fun.createFatMap(tree_mask, int(THICK_NUMBER/2.0))
    map_matrix = cv2.bitwise_or(thick_tree, thick_mask)
    cv2.imshow("FAT MAZE", thick_mask)

    # Find START coordinates
    start_x, start_y = fun.detectCenterOfMass(robot_mask)

    # Find END coordinates
    if (text == 'MAP_TO_RESCUE'):
        temp_x, temp_y = fun.detectCenterOfMass(manta_mask)
        end_x = temp_x
        end_y = temp_y + (THICK_NUMBER + 2)
    else:
        hospital_mask = fun.findHospitalMask(original)
        end_x, end_y = fun.detectCenterOfMass(hospital_mask)
        # For returning, the masta is also considered an obstacle
        thick_manta = fun.createFatMap(manta_mask, 2)
        map_matrix = cv2.bitwise_or(map_matrix, thick_manta)

    # Path Finding algorithm
    aStar_path = fun.aStar(map_matrix, start_y, start_x, end_y, end_x)

    # Check if a path exists.
    if (len(aStar_path) != 0):
        fun.pathToImage(obstacles_image, aStar_path)
        commands = fun.getCommands(aStar_path)
        return commands, True
    else:
        return [], False


def MOVING_TO_PATH(commands, gnd_robot_position, gnd_robot_angle, clientID_gnd, left_motor, right_motor, speed):
    if (fun.checkAngle(gnd_robot_angle, commands[0][2]) == True):
        # Function to check position, [x,y] x y are bool values, 1 means two positions are same, 0 means different
        # will return [1,1] when arrive the position.
        if (fun.checkPosition(gnd_robot_position[1][0], gnd_robot_position[1][1], commands[0][0], commands[0][1]) == [1, 1]):
            fun.groundMovement(
                'STOP', clientID_gnd, left_motor, right_motor, 0.0)
            commands = commands[1:]
        else:
            fun.groundMovement(
                'FORWARD', clientID_gnd, left_motor, right_motor, speed)

    else:
        error_angle = commands[0][2] - gnd_robot_angle
        # Get the delta for turning towards the desired orientation avoiding turns bigger that 180 degrees
        if error_angle > 180.0:
            error_angle = -180.0
        elif error_angle < -180.0:
            error_angle = 180.0
        delta = error_angle * 0.1

        sim.simxSetJointTargetVelocity(
            clientID_gnd, left_motor, 1.0 - delta, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID_gnd, right_motor, 1.0 + delta, sim.simx_opmode_oneshot)

    return commands


def SEARCHING_BEAR(tshirt_x, unique_values, clientID, left_motor, right_motor, prox_sensor):
    # Function returns isRescueDone, isReadyToSearch
    # Calculated offset
    delta = fun.controllerMove(tshirt_x)

    if (len(unique_values) == 1) and (unique_values[0] == 0):
        fun.groundMovement(
            'TURN_RIGHT', clientID, left_motor, right_motor, TURNING_SPEED)

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
