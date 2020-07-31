#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 20:31:07 2020

@author: miguel-asd
"""

import numpy as np
import time
import csv
 
from src.kinematic_model import robotKinematics
from src.joystick import Joystick
from src.serial_com import ArduinoSerial
from src import angleToPulse
from src.gaitPlanner import trotGait
from src.CoM_stabilization import stabilize


##This part of code is just to save the raw telemetry data.
fieldnames = ["t","roll","pitch"]
with open('telemetry/data.csv','w') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames = fieldnames)
    csv_writer.writeheader()

def update_data():
    with open('telemetry/data.csv','a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
        info = {"t" :  t,
                "roll" : realRoll,
                "pitch" : realPitch}
        csv_writer.writerow(info)




robotKinematics = robotKinematics()
joystick = Joystick('/dev/input/event1') #need to specify the event route
arduino = ArduinoSerial('/dev/ttyACM0') #need to specify the serial port
trot = trotGait() 
control = stabilize()
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
Ydist = 0.18
Xdist = 0.25
height = 0.16
#body frame to foot frame vector (0.08/-0.11 , -0.07 , -height)
bodytoFeet0 = np.matrix([[ 0.085 , -0.075 , -height],
                         [ 0.085 ,  0.075 , -height],
                         [-0.11 , -0.075 , -height],
                         [-0.11 ,  0.075 , -height]])

orn = np.array([0. , 0. , 0.])
pos = np.array([0. , 0. , 0.])
Upid_yorn = [0.]
Upid_y = [0.]
Upid_xorn = [0.]
Upid_x = [0.]
startTime = time.time()
lastTime = startTime
t = []
       
        
T = 0.4 #period of time (in seconds) of every step
offset = np.array([0. , 0.5 , 0.5 , 0.]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
                                         # [0. , 0.25 , 0.75 , 0.5] creep gait
interval = 0.030
for k in range(100000000000):
    if (time.time()-lastTime >= interval):
        loopTime = time.time() - lastTime
        lastTime = time.time()
        t = time.time() - startTime
        
        commandPose , commandOrn , V , angle , Wrot , T , compliantMode = joystick.read()    
        arduinoLoopTime , Xacc , Yacc , realRoll , realPitch = arduino.serialRecive()#recive serial message
        
        forceModule , forceAngle , Vcompliant , collision = control.bodyCompliant(Xacc , Yacc , compliantMode)
            
        #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
        bodytoFeet  = trot.loop(V + Vcompliant , angle + forceAngle , Wrot , T , offset , bodytoFeet0)
        
        #####################################################################################
        #####   kinematics Model: Input body orientation, deviation and foot position    ####
        #####   and get the angles, neccesary to reach that position, for every joint    ####
        FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn + commandOrn, pos + commandPose , bodytoFeet)
        pulsesCommand = angleToPulse.convert(FR_angles, FL_angles, BR_angles, BL_angles)
            
        arduino.serialSend(pulsesCommand)#send serial command to arduino
        
#         Upid_x , Upid_y , errorX , errorY , Upid_xorn , Upid_yorn = control.centerPoint(realPitch , realRoll)
#         orn[0] = Upid_xorn
#         orn[1] = Upid_yorn
#         pos[0] = Upid_x
#         pos[1] = Upid_y
        print (loopTime , arduinoLoopTime , realRoll , realPitch)
        
        update_data()
