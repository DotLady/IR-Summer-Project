import sim
import cv2
import numpy as np
import Main_Functions as fun
import Main_States as states

FORWARD_SPEED = 1.0


# -------------------------------------- START PROGRAM --------------------------------------
# Start Program and just in case, close all opened connections
print('Program started')
sim.simxFinish(-1)

# Connect to simulator running on localhost
# V-REP runs on port 19997, a script opens the API on port 19999
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

# Connect to the simulation
if (clientID != -1):
    print('Connected to remote API server')

    # Get handles to simulation objects
    print('Obtaining handles of simulation objects...')

    # Ground robot body
    res, body_gnd = sim.simxGetObjectHandle(
        clientID, 'Cuboid', sim.simx_opmode_oneshot_wait)
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

    # Start main control loop
    print('Starting...')
    delta = 0.0

    while (sim.simxGetConnectionId(clientID) != -1):
        sim.simxSetJointTargetVelocity(
            clientID, left_motor, FORWARD_SPEED + delta, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(
            clientID, right_motor, FORWARD_SPEED - delta, sim.simx_opmode_oneshot)


else:
    print('Could not connect to remote API server')

# Close all simulation elements
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
