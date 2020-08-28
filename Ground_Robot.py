''' Main ground robot file '''

import sim
import cv2
import numpy as np
import Main_Functions as main_fun
import Quadcopter_Functions as fun
import Ground_Functions as fun_gnd


IMG_WIDTH = 512
IMG_HEIGHT = 512
<<<<<<< Updated upstream
delta = 0.0
lastLeftWheelPosition = 0.0
lastRightWheelPosition = 0.0
leftWheelOdom = 0.0
onenunitdistance = 6.283185307179586
groundrobot_forwardspeed = 5.0
pathlist = [[-5.0, -7.0, 0.0],
            [-5.0, -5.0 ,90.0],
            [-4.0, -5.0, 0.0]]
=======
SPEED = 5.0
delta = 0.0

>>>>>>> Stashed changes

# -------------------------------------- START PROGRAM --------------------------------------
# Start Program and just in case, close all opened connections
print('Program started')
sim.simxFinish(-1)

# Connect to simulator running on localhost
clientID = sim.simxStart('127.0.0.1', 19998, True, True, 5000, 5)

# Connect to the simulation
if clientID != -1:
    print('Connected to remote API server')

    # Get handles to simulation objects
    print('Obtaining handles of simulation objects...')

    # Ground robot's perspective vision sensor
    res, camera = sim.simxGetObjectHandle(
<<<<<<< Updated upstream
        clientID, 'ground_vision_sensor', sim.simx_opmode_oneshot_wait)
=======
        clientID, 'Ground_vision_sensor', sim.simx_opmode_oneshot_wait)
>>>>>>> Stashed changes
    if res != sim.simx_return_ok:
        print('Could not get handle to Camera')

    # Floor proximity ir sensor
<<<<<<< Updated upstream
    res, ir_sensor = sim.simxGetObjectHandle(
        clientID, 'ground_IR_sensor', sim.simx_opmode_oneshot_wait)
=======
    res, prox_sensor = sim.simxGetObjectHandle(
        clientID, 'proximity_sensor', sim.simx_opmode_oneshot_wait)
>>>>>>> Stashed changes
    if res != sim.simx_return_ok:
        print('Could not get handle to Proximity Sensor')

    # Ground robot body
    res, body = sim.simxGetObjectHandle(
        clientID, 'Shape', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

<<<<<<< Updated upstream
=======
    # Ground robot arm joints
    res, left_joint = sim.simxGetObjectHandle(
        clientID, 'LeftJoint', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

    res, right_joint = sim.simxGetObjectHandle(
        clientID, 'RightJoint', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Robot')

>>>>>>> Stashed changes
    # Wheel drive motors
    res, leftMotor = sim.simxGetObjectHandle(
        clientID, 'leftMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to leftMotor')
    res, rightMotor = sim.simxGetObjectHandle(
        clientID, 'rightMotor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to rightMotor')

    # Wheels
    res, leftWheel = sim.simxGetObjectHandle(
        clientID, 'leftWheel', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to leftWheel')
    res, rightWheel = sim.simxGetObjectHandle(
        clientID, 'rightWheel', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to rightWheel')

    # Floor
    res, floor = sim.simxGetObjectHandle(
        clientID, 'ResizableFloor_5_25', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        print('Could not get handle to Floor')
<<<<<<< Updated upstream
        

=======
>>>>>>> Stashed changes


# -------------------------------------- Main Control Loop --------------------------------------

    # Start main control loop
    print('Starting control loop')

    res, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera, 0, sim.simx_opmode_streaming)

<<<<<<< Updated upstream
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, 5.0, sim.simx_opmode_oneshot)
        
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, 5.0, sim.simx_opmode_oneshot)
    
    #Ground robot move forward and get the diameter
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, groundrobot_forwardspeed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, groundrobot_forwardspeed, sim.simx_opmode_oneshot)
    leftWheelDiameter = \
        sim.simxGetObjectFloatParameter(clientID, leftWheel, 18, sim.simx_opmode_oneshot)[1] \
        - sim.simxGetObjectFloatParameter(clientID, leftWheel, 15, sim.simx_opmode_oneshot)[1]

    while (sim.simxGetConnectionId(clientID) != -1):
        # Get image from Camera
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_buffer)

        # unique_values, delta = main_fun.currentVisionState(
        #     'FINDING_TEDDY', res, resolution, image_gnd)
        tshirt_x = IMG_WIDTH/2
        #Function of wheel odometery
        leftWheelPosition = sim.simxGetJointPosition(clientID, leftMotor, sim.simx_opmode_oneshot)[1]
        dTheta = leftWheelPosition - lastLeftWheelPosition
        if dTheta >= np.pi:
            dTheta -= 2*np.pi
        elif dTheta <np.pi:
            dTheta += 2*np.pi
        leftWheelOdom += dTheta * leftWheelDiameter / 2
        lastLeftWheelPosition = leftWheelPosition
        
        #Function that used to test the wheelOdom reading for moving 1 unit
        # if -6.0<=groundrobotposition[1][0] <=-5.9:
        #     sim.simxSetJointTargetVelocity(
        #         clientID, leftMotor, 0.0, sim.simx_opmode_oneshot)
        #     sim.simxSetJointTargetVelocity(
        #         clientID, rightMotor, 0.0, sim.simx_opmode_oneshot)
        #
        #Update the position
        groundrobotposition = sim.simxGetObjectPosition(clientID, body, floor, sim.simx_opmode_blocking)
        groundrobotangle_Euler = sim.simxGetObjectOrientation(clientID , body, floor, sim.simx_opmode_blocking)[1][2]
        groundrobotangle = fun_gnd.changeangletoeurrle(groundrobotangle_Euler)
        print(groundrobotangle)
        #If get a path list
        if (len(pathlist) >=1):
            # Check if the angle of ground robot is same as the list, if true check position else robot rotating
            if (fun_gnd.checkangle(groundrobotangle,pathlist[0][2])==True):
                sim.simxSetJointTargetVelocity(clientID, leftMotor, 0.0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, rightMotor, 0.0, sim.simx_opmode_oneshot)
                groundrobotposition = sim.simxGetObjectPosition(clientID, body, floor, sim.simx_opmode_blocking)
                groundrobotangle_Euler = sim.simxGetObjectOrientation(clientID , body, floor, sim.simx_opmode_blocking)[1][2]
                groundrobotangle = fun_gnd.changeangletoeurrle(groundrobotangle_Euler)
                #Function to check position, [x,y] x y are bool values, 1 means two positions are same, 0 means different
                #will return [1,1] when arrive the position.
                if (fun_gnd.checkposition(groundrobotposition[1][0],groundrobotposition[1][1],pathlist[0][0],pathlist[0][1])==[1,1]):
                    sim.simxSetJointTargetVelocity(clientID, leftMotor, 0.0, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(clientID, rightMotor, 0.0, sim.simx_opmode_oneshot)
                    groundrobotposition = sim.simxGetObjectPosition(clientID, body, floor, sim.simx_opmode_blocking)
                    groundrobotangle_Euler = sim.simxGetObjectOrientation(clientID , body, floor, sim.simx_opmode_blocking)[1][2]
                    groundrobotangle = fun_gnd.changeangletoeurrle(groundrobotangle_Euler)
                    del(pathlist[0])
                else:
                    sim.simxSetJointTargetVelocity(clientID, leftMotor, 5.0, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(clientID, rightMotor, 5.0, sim.simx_opmode_oneshot)
                    groundrobotposition = sim.simxGetObjectPosition(clientID, body, floor, sim.simx_opmode_blocking)
                    groundrobotangle_Euler = sim.simxGetObjectOrientation(clientID , body, floor, sim.simx_opmode_blocking)[1][2]
                    groundrobotangle = fun_gnd.changeangletoeurrle(groundrobotangle_Euler)
            else:
                errorangle = pathlist[0][2]-groundrobotangle
                rotatespeed = errorangle * 0.5
                rotatespeed = fun_gnd.deltaspeed(rotatespeed)
                sim.simxSetJointTargetVelocity(clientID, leftMotor, 5.0 - rotatespeed, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, rightMotor, 5.0 + rotatespeed, sim.simx_opmode_oneshot)
                groundrobotposition = sim.simxGetObjectPosition(clientID, body, floor, sim.simx_opmode_blocking)
                groundrobotangle_Euler = sim.simxGetObjectOrientation(clientID , body, floor, sim.simx_opmode_blocking)[1][2]
                groundrobotangle = fun_gnd.changeangletoeurrle(groundrobotangle_Euler)
        # if res == sim.simx_return_ok:
        #     original = np.array(image, dtype=np.uint8)
        #     original.resize([resolution[0], resolution[1], 3])
        #     original = cv2.flip(original, 0)
        #     original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

        #     # Find mask for Mr York's tshirt
        #     tshirt_mask = fun_gnd.findBearMask(original)

        #     values = list(tshirt_mask)
        #     unique_values = np.unique(values)

        #     # Finding centre of mass of Mr York's tshirt
        #     tshirt_image, tshirt_x, tshirt_y = fun.detectCenterOfMass(
        #         tshirt_mask, True)
        #     cv2.imshow("Center_of_Tshirt", tshirt_image)
=======
    isRescueDone = False
    isReadyToSearch = False
    unique_values = []

    while (sim.simxGetConnectionId(clientID) != -1):
        # Get image from Camera
        res_camera, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_buffer)
        res_left_joint, left_joint_pos = sim.simxGetJointPosition(
            clientID, left_joint, sim.simx_opmode_oneshot)
        res_right_joint, right_joint_pos = sim.simxGetJointPosition(
            clientID, right_joint, sim.simx_opmode_oneshot)

        if (not isRescueDone):
            if (left_joint_pos < (np.pi-0.15)) and (right_joint_pos < 0.15):
                if (res_left_joint == sim.simx_return_ok) and (res_right_joint == sim.simx_return_ok):
                    sim.simxSetJointTargetVelocity(
                        clientID, left_joint, 0.2, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(
                        clientID, right_joint, -0.2, sim.simx_opmode_oneshot)
            else:
                sim.simxSetJointTargetVelocity(
                    clientID, left_joint, 0.0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID, right_joint, 0.0, sim.simx_opmode_oneshot)
                isReadyToSearch = True
        else:
            # if (left_joint_pos > 0):
            if (res_left_joint == sim.simx_return_ok) and (res_right_joint == sim.simx_return_ok):
                sim.simxSetJointTargetVelocity(
                    clientID, left_joint, -0.2, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID, right_joint, 0.2, sim.simx_opmode_oneshot)
            # else:
            #     sim.simxSetJointTargetVelocity(
            #         clientID, left_joint, 0.0, sim.simx_opmode_oneshot)
            #     sim.simxSetJointTargetVelocity(
            #         clientID, right_joint, 0.0, sim.simx_opmode_oneshot)

        # unique_values, delta = main_fun.currentVisionState(
        #     'FINDING_TEDDY', res, resolution, image_gnd)
        tshirt_x = IMG_WIDTH/2.0

        if (res_camera == sim.simx_return_ok) and isReadyToSearch:
            original = np.array(image, dtype=np.uint8)
            original.resize([resolution[0], resolution[1], 3])
            original = cv2.flip(original, 0)
            original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)

            # Find mask for Mr York's tshirt
            tshirt_mask = fun_gnd.findBearMask(original)

            values = list(tshirt_mask)
            unique_values = np.unique(values)

            # Finding centre of mass of Mr York's tshirt
            tshirt_image, tshirt_x, tshirt_y = fun_gnd.detectCenterOfMass(
                tshirt_mask, True)
            # cv2.imshow("Center_of_Tshirt", tshirt_image)
>>>>>>> Stashed changes

        # elif res == sim.simx_return_novalue_flag:
        #     # Camera has not started or is not returning images
        #     print("Wait, there's no image yet")
        # else:
        #     # Something else has happened
        #     print("Unexpected error returned", res)

<<<<<<< Updated upstream
        # # Calculated offset
        # delta = fun_gnd.controllerMove(tshirt_x)

        # # Look up for Mr York and move towards him
        # if (len(unique_values) == 1):
        #     if (unique_values[0] == 0):
        #         fun_gnd.groundMovement(
        #             'TURN_RIGHT', clientID, leftMotor, rightMotor, delta)
        #     if (unique_values[0] == 255):
        #         fun_gnd.groundMovement(
        #             'STOP', clientID, leftMotor, rightMotor, delta)
        # else:
        #     fun_gnd.groundMovement('FORWARD', clientID,
        #                            leftMotor, rightMotor, delta)

        # keypress = cv2.waitKey(1) & 0xFF
        # if keypress == ord('q'):
        #     break

        # # ----------- IDEA THAT REQUIRES DEVELOPMENT -----------
        # # Here the idea is to continue moving forward towards Mr York
        # # until the proximity sensor detects something.
        # res, isDetecting, point_detected, num, vector = sim.simxReadProximitySensor(
        #     clientID, ir_sensor, sim.simx_opmode_oneshot)
        # if isDetecting:
        #     print("Mr York detected")
        #     print("Point coordinates: ", point_detected)

        # '''
        # res, position = sim.simxGetObjectPosition(
        #     clientID, body_gnd, floor, sim.simx_opmode_oneshot)
        # if res == sim.simx_return_ok:
        #     print("X: ", round(position[0], 0))
        #     print("Y: ", round(position[1], 0))
        #     print("Z: ", round(position[2], 0))'''
=======
        if isReadyToSearch:
            # Calculated offset
            delta = fun_gnd.controllerMove(tshirt_x)

            # Look up for Mr York and move towards him
            if (len(unique_values) == 1):
                if (unique_values[0] == 0):
                    fun_gnd.groundMovement(
                        'TURN_RIGHT', clientID, leftMotor, rightMotor, SPEED, delta)
                elif (unique_values[0] == 255):
                    fun_gnd.groundMovement(
                        'STOP', clientID, leftMotor, rightMotor, SPEED, delta)
                    isRescueDone = True
                    isReadyToSearch = False

            else:
                fun_gnd.groundMovement('FORWARD', clientID,
                                       leftMotor, rightMotor, SPEED, delta)
                res, isDetecting, point_detected, num, vector = sim.simxReadProximitySensor(
                    clientID, prox_sensor, sim.simx_opmode_oneshot)
                if isDetecting:
                    # print("Mr York detected")
                    fun_gnd.groundMovement(
                        'STOP', clientID, leftMotor, rightMotor, SPEED, delta)
                    isRescueDone = True
                    isReadyToSearch = False

        keypress = cv2.waitKey(1) & 0xFF
        if keypress == ord('q'):
            break

>>>>>>> Stashed changes

else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
