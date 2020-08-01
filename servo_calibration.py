#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  4 14:51:58 2020

@author: miguel-asd
"""


import numpy as np
import time

from src.kinematic_model import robotKinematics
from src.joystick import Joystick
from src.serial_com import ArduinoSerial
from src import angleToPulse
from src.gaitPlanner import trotGait
from src.CoM_stabilization import stabilize

robotKinematics = robotKinematics()
arduino = ArduinoSerial('/dev/ttyACM0') #need to specify the serial port
trot = trotGait() 
control = stabilize()
joystick = Joystick('/dev/input/event1') #need to specify the event route
#robot properties
"""initial safe position"""
#angles
BR_angles = np.array([0 , 0 , -np.pi/2])#BR
BL_angles = np.array([0 , 0 , -np.pi/2])#BL
FL_angles = np.array([0 , 0 , -np.pi/2])#FL
FR_angles = np.array([0 , 0 , -np.pi/2])#FR

loopTime = 0.
interval = 0.040
while True:
    if (time.time()-loopTime >= interval):
#         print(time.time() - loopTime)
        loopTime = time.time() 
         
         
        commandPose , commandOrn , V , angle , Wrot , T , compliantMode = joystick.read()

        arduinoLoopTime , Xacc , Yacc , realRoll , realPitch = arduino.serialRecive()#recive serial message
        
        pulsesCommand = angleToPulse.convert(FR_angles, FL_angles, BR_angles, BL_angles)
            
        arduino.serialSend(pulsesCommand)#send serial command to arduino
