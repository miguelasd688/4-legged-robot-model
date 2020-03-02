#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 15:16:47 2020

@author: linux-asd
"""

import pybullet as p
import numpy as np
import time
import pybullet_data
from pybullet_debugger import pybulletDebug  
from kinematic_model import robotKinematics



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.2]
boxId = p.loadURDF("4leggedRobot.urdf",cubeStartPos, useFixedBase=False)
jointIds = []
paramIds = [] 

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

#robot properties
"""initial safe position"""
#angles
targetAngs = np.matrix([0 , np.pi/4 , -np.pi/2, 0 ,#BR
                        0 , np.pi/4 , -np.pi/2, 0 ,#BL
                        0 , np.pi/4 , -np.pi/2, 0 ,#FL
                        0 , np.pi/4 , -np.pi/2, 0 ])#FR
    
#FR_0  to FR_4 
#FRcoord = np.matrix([0. , -3.6 , -0.15])
#FLcoord = np.matrix([0. ,  3.6 , -0.15])
#BRcoord = np.matrix([0. , -3.6 , -0.15])
#BLcoord = np.matrix([0. ,  3.6 , -0.15])


"IS units (m,kg,rad...) "
L = 0.19 #length of robot joints
W = 0.11 #width of robot joints

"initial foot position"
#foot separation (0.182 -> tetta=0) and distance to floor
Ydist = 0.11
Xdist = L
height = 0.15
#body frame to coxa frame vector
#bodytoFR0 = np.array([ L/2, -W/2 , 0])
#bodytoFL0 = np.array([ L/2,  W/2 , 0])
#bodytoBR0 = np.array([-L/2, -W/2 , 0])
#bodytoBL0 = np.array([-L/2,  W/2 , 0])
#body frame to foot frame vector
bodytoFeet = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                        [ Xdist/2 ,  Ydist/2 , -height],
                        [-Xdist/2 , -Ydist/2 , -height],
                        [-Xdist/2 ,  Ydist/2 , -height]])
    

orientation = np.array([0. , 0. , 0.])
deviation = np.array([0. , 0. , 0.])

while(True):
    timeNow = time.time()

    deviation , orientation = pybulletDebug.cam_and_robotstates(boxId)
    
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, FL_angles, BR_angles, BL_angles = robotKinematics.solve(orientation , deviation , bodytoFeet)
    
    #move movable joints
    for i in range(0, footFR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    for i in range(footFR_index + 1, footFL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FL_angles[i - footFL_index])
    for i in range(footFL_index + 1, footBR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BR_angles[i - footBR_index])
    for i in range(footBR_index + 1, footBL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BL_angles[i - footBL_index])
    #compute simulation
    p.stepSimulation()
    
    lastTime = time.time()
    loopTime = lastTime - timeNow
    print(loopTime)
    time.sleep(0.005)
p.disconnect()
