'''File that includes all the functions used in the GroundRobot file'''

import sim
import cv2
import numpy as np


SPEED = 5.0
K_GAIN = 1.2
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

def changeangletoeurrle(angle):
    return angle*180 / np.pi

def checkangle(angle1,angle2):
    if angle2-1<angle1 <angle2+1:
        return True
    else:
        return False
    
def checkposition(x1,y1,x2,y2):
    xposition =0
    yposition =0
    if x2-0.5<x1<x2+0.5:
        xposition =1
    if y2-0.5<y1<y2+0.5:
        yposition =1
    return [xposition,yposition]

def deltaspeed(delta):
    if delta>=10.0:
        delta =10.0
        return delta
    if delta<=-10.0:
        delta = -10.0
        return delta
    else:
        return delta