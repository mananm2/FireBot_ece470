# -*- coding: utf-8 -*-
"""
Created on Fri Nov 29 01:41:50 2019

@author: Manan R Mehta
"""

#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
import math
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6,6))
    
    M = [[0.0, -1.0, 0.0, 540.0],[0.0, 0.0, -1.0, 192.0],\
        [1.0, 0.0, 0.0, 212.0],[0.0, 0.0, 0.0, 1.0]]
    M = np.array(M)
    S = [[0.0,0.0,0.0,0.0,1.0,0.0],\
         [0.0,1.0,1.0,1.0,0.0,1.0],\
         [1.0,0.0,0.0,0.0,0.0,0.0],\
         [0.0,-152.0,-152.0,-152.0,0.0,-152.0],\
         [0.0,0.0,0.0,0.0,152.0,0.0],\
         [0.0,0.0,244.0,457.0,-110.0,540.0]]

    S = np.array(S)

	# ==============================================================#
    return M, S
"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	# Initialize the return_value 
    return_value = [None, None, None, None, None, None]

    print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M, S = Get_MS()

    for i in range(6):
        s_skew = [[0.0, -S[2,i], S[1,i], S[3,i]],\
                   [S[2,i], 0.0, -S[0,i], S[4,i]],\
                   [-S[1,i], S[0,i], 0.0, S[5,i]],\
                   [0.0,0.0,0.0,0.0]]
        s_skew = np.array(s_skew)*theta[i]
        if i==0:
            T = expm(s_skew)
        else:
            T = np.dot(T,expm(s_skew))
    T = np.dot(T,M)
	
    return_value[0] = theta1 #+ np.pi
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 #- (0.5*np.pi)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # theta1 to theta6
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    l01 = 0.152
    l02 = 0.120
    l03 = 0.244
    l04 = 0.093
    l05 = 0.213
    l06 = 0.083
    l07 = 0.083
    l08 = 0.082    
    l09 = 0.0535
    l10 = 0.059   # thickness of aluminum plate is around 0.006
    yaw = yaw_WgripDegree*np.pi/180.0

    xgrip = xWgrip+0.15
    ygrip = yWgrip-0.15
    zgrip = zWgrip

    xcen = xgrip - l09*np.cos(yaw)
    ycen = ygrip - l09*np.sin(yaw)
    zcen = zgrip

	# theta1
    thetas[0] = math.atan2(ycen,xcen) - np.arcsin((0.027+l06)/np.sqrt(xcen**2 + ycen**2))    

	# theta6
    thetas[5] = thetas[0] + np.pi/2 - yaw 
 
    phi = math.atan2(0.027+l06,l07)	

    x3end = xcen - np.sqrt( (l07)**2 + (l06+0.027)**2 ) * np.cos(thetas[0] + np.pi/2 - phi)
    y3end = ycen - np.sqrt( (l07)**2 + (l06+0.027)**2 ) * np.sin(thetas[0] + np.pi/2 - phi)
    z3end = zcen + l10 + l08

    d = z3end - l01
    r = np.sqrt( x3end**2 + y3end**2 + d**2 )
    alpha = math.atan2(d,np.sqrt(r**2-d**2))
    beta = math.acos( (l03**2 + r**2 - l05**2)/(2*l03*r) )
    gamma = math.acos( (l03**2 + l05**2 - r**2)/(2*l03*l05) )
    
    thetas[1]= -(alpha + beta)

    thetas[2]= (np.pi - gamma)

    
    thetas[3]= (np.pi/2)-(np.pi*3.0/2) + (alpha + beta + gamma)
	
    thetas[4]= -np.pi/2   

    #return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          #float(thetas[3]), float(thetas[4]), float(thetas[5]) )
    return thetas
    
if __name__ == '__main__':
	invk(0.3, 0.0, 0.2, 0)