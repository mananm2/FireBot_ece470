# -*- coding: utf-8 -*-
"""
Created on Mon Dec 16 00:14:03 2019

@author: Manan R Mehta
"""

import vrep
import numpy as np
import time
from invk_func import invk

# Function that used to move joints
def SetJointPosition(theta):
    for i in range(len(theta)):
        vrep.simxSetJointTargetPosition(clientID, joint_handle[i], theta[i], vrep.simx_opmode_oneshot)

#Callable function Move from wall_following_bot.py
def Move(x,y,z,yaw):
    target=invk(x,y,z,yaw)
    SetJointPosition(target)
    time.sleep(1)

###############################################################################################################

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# Get handles to the all robot joints, end effector and the base link, raise exceptions if error encountered
result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for base frame')    

result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')

result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for end effector')
    
joint_handle = [joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle]
