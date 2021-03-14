#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: linux-asd
"""

import pybullet as p
import numpy as np
import time
import pybullet_data
from pybullet_debuger import pybulletDebug  
from kinematic_model import robotKinematics
from gaitPlanner import trotGait
from src.state_stimator import systemStateEstimator

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)


cubeStartPos = [0,0,0.2]
FixedBase = False #if fixed no plane is imported
if (FixedBase == False):
    p.loadURDF("plane.urdf")
boxId = p.loadURDF("4leggedRobot.urdf",cubeStartPos, useFixedBase=FixedBase)

jointIds = []
paramIds = [] 
time.sleep(0.5)
for j in range(p.getNumJoints(boxId)):
#    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(boxId, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointIds.append(j)
    
footFR_index = 3
footFL_index = 7
footBR_index = 11
footBL_index = 15   

pybulletDebug = pybulletDebug()
robotKinematics = robotKinematics()
trot = trotGait() 
meassure = systemStateEstimator(boxId) #meassure from simulation

#robot properties
"""initial foot position"""
#foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
Xdist = 0.20
Ydist = 0.15
height = 0.15
#body frame to foot frame vector
bodytoFeet0 = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                         [ Xdist/2 ,  Ydist/2 , -height],
                         [-Xdist/2 , -Ydist/2 , -height],
                         [-Xdist/2 ,  Ydist/2 , -height]])

T = 0.5 #period of time (in seconds) of every step
offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
bodytoFeet_vecX = 0.
bodytoFeet_vecY = 0.


pos = np.zeros([3])
orn = np.zeros([3])
p.setRealTimeSimulation(1)
p.setTimeStep(0.002)

for i in range(100000):
    lastTime = time.time()
    
    
    _ , _ , L , angle , Lrot , T = pybulletDebug.cam_and_robotstates(boxId)  
    #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    bodytoFeet = trot.loop(L , angle , Lrot , T , offset , bodytoFeet0)

#####################################################################################
#####   kinematics Model: Input body orientation, deviation and foot position    ####
#####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn , pos , bodytoFeet)
            
    t , X = meassure.states()
    U , Ui ,torque = meassure.controls()
    bodytoFeet_vecX = np.append(bodytoFeet_vecX , bodytoFeet[0,0])
    bodytoFeet_vecY = np.append(bodytoFeet_vecY , bodytoFeet[0,2])
    
    #move movable joints
    for i in range(3):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, 
                                targetPosition = FR_angles[i] , force = maxForce , maxVelocity = masVel)
        p.setJointMotorControl2(boxId, 4 + i, p.POSITION_CONTROL, 
                                targetPosition = FL_angles[i] , force = maxForce , maxVelocity = masVel)
        p.setJointMotorControl2(boxId, 8 + i, p.POSITION_CONTROL, 
                                targetPosition = BR_angles[i] , force = maxForce , maxVelocity = masVel)
        p.setJointMotorControl2(boxId, 12 + i, p.POSITION_CONTROL, 
                                targetPosition = BL_angles[i] , force = maxForce , maxVelocity = masVel)
     
#    print(time.time() - lastTime)
p.disconnect()
