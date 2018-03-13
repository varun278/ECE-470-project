import sys
import vrep
import time
import numpy as np

def move(dtheta1,dtheta2, dtheta3, dtheta4, dtheta5, dtheta6, dtheta7):
    # Close all open connections (just in case)
    vrep.simxFinish(-1)
    
    # Connect to V-REP (raise exception on failure)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')
    
    # Get "handle" to the joints of robot
    result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'Sawyer_joint1', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for first joint')

    result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'Sawyer_joint2', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for second joint')

    result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'Sawyer_joint3', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for third joint')

    result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'Sawyer_joint4', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for fourth joint')

    result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'Sawyer_joint5', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for fifth joint')

    result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'Sawyer_joint6', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for sixth joint')

    result, joint_seven_handle = vrep.simxGetObjectHandle(clientID, 'Sawyer_joint7', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for seventh joint')

    # Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    # Wait two seconds
    time.sleep(2)

    # Get the current value of the joint variables
    result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of first joint variable: theta1 = {:f}'.format(theta1))

    result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of second joint variable: theta2 = {:f}'.format(theta2))

    result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of third joint variable: theta3 = {:f}'.format(theta3))

    result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of fourth joint variable: theta4 = {:f}'.format(theta4))

    result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of fifth joint variable: theta5 = {:f}'.format(theta5))

    result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of sixth joint variable: theta6 = {:f}'.format(theta6))

    result, theta7 = vrep.simxGetJointPosition(clientID, joint_seven_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of seventh joint variable: theta7 = {:f}'.format(theta7))

    # Set the desired value of the first joint variable
    vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1 + dtheta1, vrep.simx_opmode_oneshot)
    # Wait one seconds
    time.sleep(1)
    # Set the desired value of the second joint variable
    vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta2 + dtheta2, vrep.simx_opmode_oneshot)
    # Wait one seconds
    time.sleep(1)
    # Set the desired value of the third joint variable
    vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3 + dtheta3, vrep.simx_opmode_oneshot)
    # Wait one seconds
    time.sleep(1)
    # Set the desired value of the fourth joint variable
    vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta4 + dtheta4, vrep.simx_opmode_oneshot)
    # Wait one seconds
    time.sleep(1)
    # Set the desired value of the fifth joint variable
    vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta5 + dtheta5, vrep.simx_opmode_oneshot)
    # Wait one seconds
    time.sleep(1)
    # Set the desired value of the sixth joint variable
    vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta6 + dtheta6, vrep.simx_opmode_oneshot)
    # Wait one seconds
    time.sleep(1)
    # Set the desired value of the seventh joint variable
    vrep.simxSetJointTargetPosition(clientID, joint_seven_handle, theta7 + dtheta7, vrep.simx_opmode_oneshot)

    # Wait 10 seconds to see the final pose
    time.sleep(5)

    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    # Close the connection to V-REP
    vrep.simxFinish(clientID)

def main():
    theta1 = float(input('Please enter a value joint 1 \n'))
    theta2 = float(input('Please enter a value joint 2 \n'))
    theta3 = float(input('Please enter a value joint 3 \n'))
    theta4 = float(input('Please enter a value joint 4 \n'))
    theta5 = float(input('Please enter a value joint 5 \n'))
    theta6 = float(input('Please enter a value joint 6 \n'))
    theta7 = float(input('Please enter a value joint 7 \n'))
    move(theta1,theta2, theta3, theta4, theta5, theta6, theta7)

if __name__=="__main__":
    main()
