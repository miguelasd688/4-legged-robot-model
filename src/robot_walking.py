#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 20:31:07 2020

@author: linux-asd
"""

import pybullet as p
import numpy as np
import time
import pybullet_data
from simple_pid import PID 

import angleToPulse
from pybullet_debuger import pybulletDebug  
from kinematic_model import robotKinematics
from joystick import Joystick
from serial_com import ArduinoSerial
from gaitPlanner import trotGait


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)


cubeStartPos = [0,0,0.3]
FixedBase = False
boxId = p.loadURDF("4leggedRobot.urdf",cubeStartPos, useFixedBase=FixedBase)
if (FixedBase == False):
    p.loadURDF("plane.urdf")
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
joystick = Joystick('/dev/input/event18') #need to specify the event route
arduino = ArduinoSerial('/dev/ttyACM1') #need to specify the serial port
trot = trotGait() 
#robot properties
"""initial safe position"""
#angles
targetAngs = np.array([0 , np.pi/4 , -np.pi/2, 0 ,#BR
                        0 , np.pi/4 , -np.pi/2, 0 ,#BL
                        0 , np.pi/4 , -np.pi/2, 0 ,#FL
                        0 , np.pi/4 , -np.pi/2, 0 ])#FR

#FR_0  to FR_4 
#FRcoord = np.matrix([0. , -3.6 , -0.15])
#FLcoord = np.matrix([0. ,  3.6 , -0.15])
#BRcoord = np.matrix([0. , -3.6 , -0.15])
#BLcoord = np.matrix([0. ,  3.6 , -0.15])


"initial foot position"
#foot separation (0.182 -> tetta=0) and distance to floor
Ydist = 0.17
Xdist = 0.25
height = 0.17
#body frame to foot frame vector
bodytoFeet0 = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                        [ Xdist/2 ,  Ydist/2 , -height],
                        [-Xdist/2 , -Ydist/2 , -height],
                        [-Xdist/2 ,  Ydist/2 , -height]])

orientation = np.array([0. , 0. , 0.])
deviation = np.array([0. , 0. , 0.])
lastTime = 0.


pidX = PID(-0.0005 , 0.0 , 0.000001 , setpoint=0.)
pidY = PID(0.0005 , 0. , 0. , setpoint=0.)
pidX.sample_time = 0.02  # update every 0.01 seconds
pidY.sample_time = 0.02  


T = 0.5 #period of time (in seconds) of every step
offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
while(True):
    startTime = time.time()
    pos , orn , L , angle , Lrot , T = pybulletDebug.cam_and_robotstates(boxId)    
    
    L , angle , Lrot , T  = joystick.read()    
    
    #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    bodytoFeet , s = trot.loop(L , angle , Lrot , T , offset , bodytoFeet0)
    
    arduinoLoopTime , realRoll , realPitch = arduino.serialRecive()#recive serial message

    pos[0] = 0.015 + pidX(realPitch)
    pos[1] = pidY(realRoll)
    
    
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn , pos , bodytoFeet)
    pulsesCommand = angleToPulse.convert(FR_angles, FL_angles, BR_angles, BL_angles)
        
    arduino.serialSend(pulsesCommand)#send serial command to arduino
        
    #move movable joints
    for i in range(0, footFR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    for i in range(footFR_index + 1, footFL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FL_angles[i - footFL_index])
    for i in range(footFL_index + 1, footBR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BR_angles[i - footBR_index])
    for i in range(footBR_index + 1, footBL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BL_angles[i - footBL_index])
    
    p.stepSimulation()#compute simulation
    
    print(time.time() - startTime ,T)
p.disconnect()


