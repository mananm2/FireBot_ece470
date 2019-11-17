# -*- coding: utf-8 -*-
"""
Created on Sun Sep 22 13:13:37 2019
@author: Manan R Mehta
"""
import vrep
import sys
import time
import numpy as np

vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Connection not successful')
    sys.exit('Could not connect')

speed = 2.5
   
errorCode1, Left_Motor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
errorCode2, Right_Motor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)

vrep.simxSetJointTargetVelocity(clientID, Left_Motor, speed, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, Right_Motor, speed, vrep.simx_opmode_oneshot)

errorCode3, leftsensor = vrep.simxGetObjectHandle(clientID,'Proximity_sensor_left',vrep.simx_opmode_blocking)
errorCode3, frontsensor = vrep.simxGetObjectHandle(clientID,'Proximity_sensor_front',vrep.simx_opmode_blocking)
errorCode3, backsensor = vrep.simxGetObjectHandle(clientID,'Proximity_sensor_back',vrep.simx_opmode_blocking)


errorCode4, IsObstacle, DistanceVector_left, ObstacleHandle, SurfaceNormalVector \
= vrep.simxReadProximitySensor(clientID,leftsensor,vrep.simx_opmode_streaming)

errorCode5, IsObstacle, DistanceVector_front, ObstacleHandle, SurfaceNormalVector \
= vrep.simxReadProximitySensor(clientID,frontsensor,vrep.simx_opmode_streaming)

k=0
dist_opt = 1

while clientID!=-1:
    errorCode4, IsObstacle_left, left, ObstacleHandle_left, SurfaceNormalVector_left \
            = vrep.simxReadProximitySensor(clientID, leftsensor ,vrep.simx_opmode_buffer)        
    errorCode5, IsObstacle_front, front, ObstacleHandle_front, SurfaceNormalVector_front \
            = vrep.simxReadProximitySensor(clientID, frontsensor ,vrep.simx_opmode_buffer)        
    
    
    if left != [0,0,0] and k == 0:
        dist_opt = np.linalg.norm(left)
        dist_org = dist_opt
        print(dist_opt)
        k=1
    
    if k==0:
        continue

    
    if np.linalg.norm(left) >= dist_opt and not IsObstacle_front:
        if np.absolute(SurfaceNormalVector_left[2]) < 0.99 or np.linalg.norm(left) > dist_org+0.25:
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, 0.6*speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 1*speed, vrep.simx_opmode_oneshot)
            print('Turning Left') 
        else:
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, speed, vrep.simx_opmode_oneshot)
            print('Going Straight')    
    
    
    elif np.linalg.norm(front) < dist_org and IsObstacle_front:
        d = np.linalg.norm(left)
        while IsObstacle_front or np.absolute(SurfaceNormalVector_left[2]) < 0.99:
            errorCode4, IsObstacle_left, left, ObstacleHandle_left, SurfaceNormalVector_left \
                = vrep.simxReadProximitySensor(clientID, leftsensor ,vrep.simx_opmode_buffer)        
            errorCode5, IsObstacle_front, front, ObstacleHandle_front, SurfaceNormalVector_front \
                = vrep.simxReadProximitySensor(clientID, frontsensor ,vrep.simx_opmode_buffer)    
            
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 0.0*speed, vrep.simx_opmode_oneshot)
            print('Turning Right')
        dist_opt = np.linalg.norm(left)
        
    
    elif (not IsObstacle_left) and (not IsObstacle_front):
        while np.absolute(SurfaceNormalVector_left[2]) < 0.99 and np.linalg.norm(left) < dist_opt:
            errorCode4, IsObstacle_left, left, ObstacleHandle_left, SurfaceNormalVector_left \
                = vrep.simxReadProximitySensor(clientID, leftsensor ,vrep.simx_opmode_buffer)        
            errorCode5, IsObstacle_front, front, ObstacleHandle_front, SurfaceNormalVector_front \
                = vrep.simxReadProximitySensor(clientID, frontsensor ,vrep.simx_opmode_buffer)    
            
            vrep.simxSetJointTargetVelocity(clientID, Left_Motor, -0.1*speed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, Right_Motor, 0.5*speed, vrep.simx_opmode_oneshot)
            print('Turning left')
        dist_opt = np.linalg.norm(left)


        

               
