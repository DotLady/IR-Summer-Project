'''File that includes all the functions used in the GroundRobot file'''

import sim
import cv2
import numpy as np


SPEED = 5.0
K_GAIN = 5.0
IMG_WIDTH = 512
IMG_HEIGHT = 512


def eulerAnglesToDegrees(euler_angle):
    angle_degrees = (euler_angle*360.0)/(2*np.pi)

    new_angle = angle_degrees

    if angle_degrees == 360.0:
        new_angle = 0.0
    elif angle_degrees < 0.0:
        new_angle += 360

    return new_angle


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

    return color_image, cX, cY


def groundMovement(state, clientID, leftMotor, rightMotor, speed, delta):
    if state == 'FORWARD':
        sim.simxSetJointTargetVelocity(
            clientID, leftMotor, speed + delta, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, rightMotor, speed - delta, sim.simx_opmode_oneshot)
    elif state == 'TURN_LEFT':
        sim.simxSetJointTargetVelocity(
            clientID, leftMotor, -speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, rightMotor, speed, sim.simx_opmode_oneshot)
    elif state == 'TURN_RIGHT':
        sim.simxSetJointTargetVelocity(
            clientID, leftMotor, speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, rightMotor, -speed, sim.simx_opmode_oneshot)
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
