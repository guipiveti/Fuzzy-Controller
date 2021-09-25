# -*- coding: utf-8 -*-
"""
Created on Sat Sep 25 10:36:55 2021

@author: guipi
"""
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 16 22:47:45 2021

@author: guipi
"""
import sim
import numpy as np
import time
import matplotlib.pyplot as plt

import skfuzzy as fuzz
from skfuzzy import control as ctrl


left = ctrl.Antecedent(np.arange(0, 101, 1), 'Left Sensor')
front_l = ctrl.Antecedent(np.arange(0, 101, 1), 'Front-L Sensor')
front_r = ctrl.Antecedent(np.arange(0, 101, 1), 'Front-R Sensor')
right = ctrl.Antecedent(np.arange(0, 101, 1), 'Right Sensor')
direction = ctrl.Consequent(np.arange(-100, 100, 1), 'Angle')

front_l['near'] = fuzz.trimf(front_l.universe, [0, 0, 5])
front_l['far'] = fuzz.trapmf(front_l.universe, [0, 5, 100, 100])
front_r['near'] = fuzz.trimf(front_r.universe, [0, 0, 5])
front_r['far'] = fuzz.trapmf(front_r.universe, [0, 5, 100, 100])
left['near'] = fuzz.trimf(left.universe, [0, 0, 3])
left['far'] = fuzz.trapmf(left.universe, [0, 3, 100, 100])
right['near'] = fuzz.trimf(right.universe, [0, 0, 3])
right['far'] = fuzz.trapmf(right.universe, [0, 3, 100, 100])

front_l.view()
front_r.view()
left.view()
right.view()

direction['Turn Left'] = fuzz.trapmf(direction.universe, [-100, -100, -20, 0])
direction['Straight'] = fuzz.trapmf(direction.universe, [-30, -10, 10, 30])
direction['Turn Right'] = fuzz.trapmf(direction.universe, [ 0, 20, 100, 100])
# direction.automf(names=["1", "2", "3"])
# direction.automf(names=["Turn Left", "Straight", "Turn Right"])

direction.view()

rule1 = ctrl.Rule(left['far'] & front_l['far'] & front_r['far'] & right['far'], direction['Straight'])
rule2 = ctrl.Rule(left['far'] & front_l['far'] & front_r['far'] & right['near'], direction['Turn Left'])
rule3 = ctrl.Rule(left['far'] & front_l['far'] & front_r['near'] & right['far'], direction['Turn Left'])
rule4 = ctrl.Rule(left['far'] & front_l['far'] & front_r['near'] & right['near'], direction['Turn Left'])

rule5 = ctrl.Rule(left['far'] & front_l['near'] & front_r['far'] & right['far'], direction['Turn Right'])
rule6 = ctrl.Rule(left['far'] & front_l['near'] & front_r['far'] & right['near'], direction['Turn Left'])
rule7 = ctrl.Rule(left['far'] & front_l['near'] & front_r['near'] & right['far'], direction['Turn Right'])
rule8 = ctrl.Rule(left['far'] & front_l['near'] & front_r['near'] & right['near'], direction['Turn Left'])

rule9 = ctrl.Rule(left['near'] & front_l['far'] & front_r['far'] & right['far'], direction['Turn Right'])
rule10 = ctrl.Rule(left['near'] & front_l['far'] & front_r['far'] & right['near'], direction['Straight'])
rule11 = ctrl.Rule(left['near'] & front_l['far'] & front_r['near'] & right['far'], direction['Turn Right'])
rule12 = ctrl.Rule(left['near'] & front_l['far'] & front_r['near'] & right['near'], direction['Turn Left'])

rule13 = ctrl.Rule(left['near'] & front_l['near'] & front_r['far'] & right['far'], direction['Turn Right'])
rule14 = ctrl.Rule(left['near'] & front_l['near'] & front_r['far'] & right['near'], direction['Turn Right'])
rule15 = ctrl.Rule(left['near'] & front_l['near'] & front_r['near'] & right['far'], direction['Turn Right'])
rule16 = ctrl.Rule(left['near'] & front_l['near'] & front_r['near'] & right['near'], direction['Turn Left'])


# rule1 = ctrl.Rule(front_l['far'] & front_r['far'], direction['Straight'])
# rule2 = ctrl.Rule(left['near'] & front_l['near'], direction['Turn Right'])
# rule3 = ctrl.Rule(right['near'] & front_r['near'], direction['Turn Left'])

# rule1.view()
# rule2.view()
# rule3.view()
# rule4.view()

# rule5.view()
# rule6.view()
# rule7.view()
# rule8.view()

# rule9.view()
# rule10.view()
# rule11.view()
# rule12.view()

# rule13.view()
# rule14.view()
# rule15.view()
# rule16.view()

# direction_ctrl = ctrl.ControlSystem(rules=[rule1, rule2, rule3])
# direction_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16])
direction_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16])
required_direction = ctrl.ControlSystemSimulation(direction_ctrl)

















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

        
        print(Ultra_Sensor_Detected_Position)
        print('\n')

        distance = np.linalg.norm(Ultra_Sensor_Detected_Position, axis=1)
        print(distance)
        print('\n')
        
        
        
        
        
        
        
        
        # Fuzzy
        required_direction.input['Left Sensor'] = distance[0]
        required_direction.input['Front-L Sensor'] = distance[2]
        required_direction.input['Front-R Sensor'] = distance[5]
        required_direction.input['Right Sensor'] = distance[7]
        
        required_direction.compute()
        print ('Direction: '+ str(required_direction.output['Angle']))
        direction.view(sim=required_direction)


        right_speed = 0;
        left_speed = 0;
        
        if required_direction.output['Angle']<0:
            right_speed = 100
            left_speed = required_direction.output['Angle']+100
        else:
            left_speed = 100
            right_speed = (-1*required_direction.output['Angle'])+100
            

        sim.simxSetJointTargetVelocity(clientID, motorRight, right_speed/30, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motorLeft, left_speed/30, sim.simx_opmode_streaming)
        
        print (required_direction.output['Angle'])
        # direction.view(sim=required_direction)


        time.sleep(0.05)


    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')