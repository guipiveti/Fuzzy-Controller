# -*- coding: utf-8 -*-
"""
Created on Thu Sep 16 22:47:45 2021

@author: guipi
"""
import sim
import numpy as np
import time
import matplotlib.pyplot as plt

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    
    errorCodeMotorLeft, motorLeft = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    errorCodeMotorRight, motorRight = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    
    
    
# Creating sensors
    Ultra_Sensor=np.zeros((16,2))
    Ultra_Sensor_Detected_Position=np.zeros((16,3))
    Ultra_Sensor_Distance=np.zeros((16))
    for i in range(1, 17):
        Ultra_Sensor[i-1]=sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor"+str(i), sim.simx_opmode_blocking)
        returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector= sim.simxReadProximitySensor(clientID, int(Ultra_Sensor[i-1][1]), sim.simx_opmode_streaming)
        if detectionState:
            Ultra_Sensor_Detected_Position[i-1]=detectedPoint
        else:
            Ultra_Sensor_Detected_Position[i-1]=[100,100,100]  
    time.sleep(2)
 
    
 # Reading sensors
    while 1:
        for i in range(1, 17):
            returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector= sim.simxReadProximitySensor(clientID, int(Ultra_Sensor[i-1][1]), sim.simx_opmode_buffer)
            if detectionState:
                Ultra_Sensor_Detected_Position[i-1]=detectedPoint
            else:
                Ultra_Sensor_Detected_Position[i-1]=[100,100,100]

        
        
        time.sleep(5)
        print(Ultra_Sensor_Detected_Position)
        print('\n')  

        distance = np.linalg.norm(Ultra_Sensor_Detected_Position, axis=1)
        print(distance)
        print('\n') 



    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')