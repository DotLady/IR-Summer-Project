'''File that includes all the functions used in the GroundRobot file'''

import sim
import cv2
import numpy as np
import matplotlib as mpl
import Quadcopter_Functions as fun
from matplotlib import pyplot as plt
from matplotlib import colors

IMG_WIDTH = 512
IMG_HEIGHT = 512


def findTeddy(original):
    hsv_image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

    # Green tshirt colour
    greenMin_tshirt = np.array([40, 120, 20])
    greenMax_tshirt = np.array([80, 255, 255])
    tshirt_mask = cv2.inRange(hsv_image, greenMin_tshirt, greenMax_tshirt)

    return tshirt_mask
