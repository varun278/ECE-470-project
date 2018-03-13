import sys
import vrep
import time
import numpy as np
import math as m
from scipy.linalg import expm, sinm, cosm
from move import move

#Written by Varun Jain
#NetID: vjain10
#ECE470 semester project
#Working in VREP with Sawyer from Rethink robotics

def forward_kin(theta1, theta2, theta3, theta4, theta5, theta6, theta7):
    #forward kinematics calculations based on function inputs
    
    #initial default position of the sawyer robot in vrep
    M = np.array([[0 , -1, 0, 0.0735], [1, 0, 0, 0.1604], [0, 0, 1, 1.2914], [0, 0, 0,1]])
    
    #joint rotation axis and position definition based on attached forward kinematics derivation (ForwardKinematics.pdf)
    a1 = np.array([[0], [0], [1]])
    a2 = np.array([[0], [1], [0]])
    a3 = np.array([[0], [0], [1]])
    a4 = np.array([[0], [1], [0]])
    a5 = np.array([[0], [0], [1]])
    a6 = np.array([[0], [1], [0]])
    a7 = np.array([[0], [0], [1]])
    q1 = np.array([[0.00012], [0.00012], [0.0865]])
    q2 = np.array([[0.0811], [0.0611], [0.3170]])
    q3 = np.array([[0.0811], [0.1926], [0.4415]])
    q4 = np.array([[0.0811], [0.1451], [0.7170]])
    q5 = np.array([[0.0811], [0.0241], [0.8301]])
    q6 = np.array([[0.0811], [0.0646], [1.1170]])
    q7 = np.array([[0.0811], [0.1604], [1.1910]])
    
    #Computation of screw axes based on the above defined a and q values (s = [a; -[a]*q] for revolute joints
    s1 = np.append(a1, -skew(a1)*q1, axis=0)
    s2 = np.append(a2, -skew(a2)*q2, axis=0)
    s3 = np.append(a3, -skew(a3)*q3, axis=0)
    s4 = np.append(a4, -skew(a4)*q4, axis=0)
    s5 = np.append(a5, -skew(a5)*q5, axis=0)
    s6 = np.append(a6, -skew(a6)*q6, axis=0)
    s7 = np.append(a7, -skew(a7)*q7, axis=0)
    
    #Computation of final pose of Sawyer using matrix exponential method
    T_1in0 = expm(skew6(s1)*theta1)@expm(skew6(s2)*theta2)@expm(skew6(s3)*theta3)@expm(skew6(s4)*theta4)@expm(skew6(s5)*theta5)@expm(skew6(s6)*theta6)@expm(skew6(s7)*theta7)@M
    
    #print an return the computed final pose
    print('\n T_1in0:\n', T_1in0)
    return T_1in0

def skew(w):
    #bracket of 3x1 vector (returns a 3x3 matrix)
    return np.matrix([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

def skew6(v):
    #bracket of 6x1 vectors (returns a 4x4 matrix)
    return np.block([[skew(v[0:3, :]), v[3:6, :]], [0, 0, 0, 0]])

def main():
    #write the main function here that calls a forward kinematic function

    #input taken from user for 7 joint angles
    theta1 = float(input('Please enter a value joint 1 \n'))
    theta2 = float(input('Please enter a value joint 2 \n'))
    theta3 = float(input('Please enter a value joint 3 \n'))
    theta4 = float(input('Please enter a value joint 4 \n'))
    theta5 = float(input('Please enter a value joint 5 \n'))
    theta6 = float(input('Please enter a value joint 6 \n'))
    theta7 = float(input('Please enter a value joint 7 \n'))
    
    #compute the final pose using these 7 joint values
    T_1in0 = forward_kin(theta1, theta2, theta3, theta4, theta5, theta6, theta7)
    
    #get the vrep clientID to communicate with vrep
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')
    
    #get the object handle for the dummy object
    result, dummy = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for first joint')

    #compute euler angles to orient the dummy object using the rotation matrix embedded in T_1in0
    euAngle1 = -m.asin(T_1in0[2][0])
    euAngle2 = m.atan2(T_1in0[2][1]/m.cos(euAngle1), T_1in0[2][2]/m.cos(euAngle1))
    euAngle3 = m.atan2(T_1in0[1][0]/m.cos(euAngle1), T_1in0[0][0]/m.cos(euAngle1))

    print('\n Euler angles:', euAngle1, euAngle2, euAngle3, '\n')

    #set the position and orientation of a dummy object to the computed orientation and position of end effector
    # to verify the computation
    vrep.simxSetObjectOrientation(clientID, dummy, -1, [euAngle1, euAngle2, euAngle3], vrep.simx_opmode_oneshot)
    time.sleep(2)
    vrep.simxSetObjectPosition(clientID, dummy, -1, [T_1in0[0][3],T_1in0[1][3] , T_1in0[2][3]], vrep.simx_opmode_oneshot)
    time.sleep(2)

    # move sawyer with the given theta inputs
    move(theta1, theta2, theta3, theta4, theta5, theta6, theta7)

if __name__=="__main__":
    main()
