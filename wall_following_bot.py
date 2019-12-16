# -*- coding: utf-8 -*-
"""
Created on Sun Sep 22 13:13:37 2019
@author: Manan R Mehta
"""
import vrep
import sys
import time
import numpy as np
import arm_mover

vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Connection not successful')
    sys.exit('Could not connect')

speed = 2.5

#Get Handles to the wheel motors   
errorCode, Left_Motor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
errorCode, Right_Motor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)

#Get handles to the proximity and vision sensors
errorCode, leftsensor = vrep.simxGetObjectHandle(clientID,'Proximity_sensor_left',vrep.simx_opmode_blocking)
errorCode, frontsensor = vrep.simxGetObjectHandle(clientID,'Proximity_sensor_front',vrep.simx_opmode_blocking)

errorCode, frontcam = vrep.simxGetObjectHandle(clientID,'frontcam',vrep.simx_opmode_blocking)
errorCode, rightcam = vrep.simxGetObjectHandle(clientID,'rightcam',vrep.simx_opmode_blocking)


#############################################################################################################

#Initialize robot motion by assigning wheel velocity to each motor
vrep.simxSetJointTargetVelocity(clientID, Left_Motor, speed, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, Right_Motor, speed, vrep.simx_opmode_oneshot)

#Initialize sensors
errorCode, IsObstacle, DistanceVector_left, ObstacleHandle, SurfaceNormalVector = vrep.simxReadProximitySensor(clientID,leftsensor,vrep.simx_opmode_streaming)
errorCode, IsObstacle, DistanceVector_front, ObstacleHandle, SurfaceNormalVector = vrep.simxReadProximitySensor(clientID,frontsensor,vrep.simx_opmode_streaming)
errorCode, detection, auxPackets = vrep.simxReadVisionSensor(clientID,frontcam,vrep.simx_opmode_streaming)
errorCode, detection, auxPackets = vrep.simxReadVisionSensor(clientID,rightcam,vrep.simx_opmode_streaming)

returnCode,resolution,buffer=vrep.simxGetVisionSensorDepthBuffer(clientID,frontcam,vrep.simx_opmode_streaming)
returnCode,resolution,buffer=vrep.simxGetVisionSensorDepthBuffer(clientID,rightcam,vrep.simx_opmode_streaming)

##############################################################################################################

k=0
dist_opt = 1

while clientID!=-1:
    errorCode, IsObstacle_left, left, ObstacleHandle_left, SurfaceNormalVector_left = vrep.simxReadProximitySensor(clientID, leftsensor ,vrep.simx_opmode_buffer)        
    errorCode, IsObstacle_front, front, ObstacleHandle_front, SurfaceNormalVector_front = vrep.simxReadProximitySensor(clientID, frontsensor ,vrep.simx_opmode_buffer)        
    
    
    if left != [0,0,0] and k == 0:
        dist_opt = np.linalg.norm(left)
        dist_org = dist_opt
        k=1
    #Exception case in which the sensor does not read distance in the first iteration!
    if k==0:
        continue

    #Straight Line Motion Condition
    if np.linalg.norm(left) >= dist_opt and not IsObstacle_front:
        if np.absolute(SurfaceNormalVector_left[2]) < 0.99 or np.linalg.norm(left) > dist_org+0.25:
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, 0.6*speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 1*speed, vrep.simx_opmode_oneshot)
            #print('No Wall! Turning Left') 
        else:
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, speed, vrep.simx_opmode_oneshot)
            #print('Following Wall! Going Straight')    
    
    #Right Turn
    elif np.linalg.norm(front) < dist_org and IsObstacle_front:
        d = np.linalg.norm(left)
        while IsObstacle_front or np.absolute(SurfaceNormalVector_left[2]) < 0.99:
            errorCode, IsObstacle_left, left, ObstacleHandle_left, SurfaceNormalVector_left = vrep.simxReadProximitySensor(clientID, leftsensor ,vrep.simx_opmode_buffer)        
            errorCode, IsObstacle_front, front, ObstacleHandle_front, SurfaceNormalVector_front = vrep.simxReadProximitySensor(clientID, frontsensor ,vrep.simx_opmode_buffer)    
            
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 0.0*speed, vrep.simx_opmode_oneshot)
            #print('Obstacle detected! Turning Right')
        dist_opt = np.linalg.norm(left)
        
    #Left Turn or Turn around a wall by 180deg
    elif (not IsObstacle_left) and (not IsObstacle_front):
        while np.absolute(SurfaceNormalVector_left[2]) < 0.99 and np.linalg.norm(left) < dist_opt:
            errorCode, IsObstacle_left, left, ObstacleHandle_left, SurfaceNormalVector_left = vrep.simxReadProximitySensor(clientID, leftsensor ,vrep.simx_opmode_buffer)        
            errorCode, IsObstacle_front, front, ObstacleHandle_front, SurfaceNormalVector_front = vrep.simxReadProximitySensor(clientID, frontsensor ,vrep.simx_opmode_buffer)    
            
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, -0.1*speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 0.5*speed, vrep.simx_opmode_oneshot)
            #print('Turning left')
        dist_opt = np.linalg.norm(left)
    
    #At the end of each loop, we check if the fire is detected using the vision sensor readings
    #If fire is detected, stop the robot motion
    errorCode, detection, colorarray_list_front = vrep.simxReadVisionSensor(clientID,frontcam,vrep.simx_opmode_buffer)
    errorCode, detection, colorarray_list_right = vrep.simxReadVisionSensor(clientID,rightcam,vrep.simx_opmode_buffer)

    colorarray_usable_front = np.array(colorarray_list_front)
    colorarray_usable_right = np.array(colorarray_list_right)
    

    if colorarray_usable_front[0][6] == 1.0 or colorarray_usable_right[0][6] == 1.0:
        speed = 0
        vrep.simxSetJointTargetVelocity(clientID, Left_Motor, 0, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 0, vrep.simx_opmode_oneshot)
        break

##############################################################################################################

#At this point, fire has been detected either in the front cam or the right cam
#Move the UR3 arm to the base of the fire
if colorarray_usable_front[0][6] == 1.0:
    returnCode, resolution, buffer = vrep.simxGetVisionSensorDepthBuffer(clientID,frontcam,vrep.simx_opmode_buffer)
    #move the ur3 towards the front
    arm_mover.Move(0.4,0.15,-0.1,1.57)
    time.sleep(1)
    
elif colorarray_usable_right[0][6] == 1.0:        
    returnCode, resolution, buffer = vrep.simxGetVisionSensorDepthBuffer(clientID,rightcam,vrep.simx_opmode_buffer)
    #buffer = sorted(buffer)
    #move the ur3 towards right and distance depends on buffer median
    arm_mover.Move(0.0,-0.4,-0.1,1.57)
    time.sleep(1) 

        

               
