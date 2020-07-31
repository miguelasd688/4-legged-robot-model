#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 14:05:02 2020

@author: miguel-asd
"""

from evdev import InputDevice, categorize, ecodes
from select import select
import numpy as np

class Joystick:
    def __init__(self , event):
        #python3 /usr/local/lib/python3.8/dist-packages/evdev/evtest.py for identify event
        self.gamepad = InputDevice(event)
        self.L3 = np.array([0. , 0.])
        self.R3 = np.array([0. , 0.])
        
        self.x=0
        self.triangle=0
        self.circle=0
        self.square=0
        
        self.T = 0.4
        self.V = 0.
        self.angle = 0.
        self.Wrot = 0.
        self.compliantMode = False
        self.poseMode = False
        self.CoM_pos = np.zeros(3)
        self.CoM_orn = np.zeros(3)
        self.calibration = 0
    def read(self):
        r,w,x = select([self.gamepad.fd], [], [], 0.)
        
        if r:
            for event in self.gamepad.read():
#                print(event)
                if event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        if event.code == 544:#up arrow
                            self.CoM_pos[2] += 0.002
                        if event.code == 545:#down arrow
                            self.CoM_pos[2] -= 0.002
                        if event.code == 547:#right arrow
                            self.T += 0.05
                        if event.code == 546:#left arrow
                            self.T -= 0.05   
                        if event.code == 308:#square
                            if self.compliantMode == True:
                                self.compliantMode = False
                            elif self.compliantMode == False:
                                self.compliantMode = True  
                        if event.code == 307:#triangle
                            if self.poseMode == True:
                                self.poseMode = False
                            elif self.poseMode == False:
                                self.poseMode = True  
                        if event.code == 310:#R1
                            self.calibration += 5
#                        if event.code == 313:#R2
#                            self.calibration -= 0.0005
#                            
                        if event.code == 311:#L1
                            self.calibration -= 5
#                        if event.code == 312:#L2
#                            self.CoM_orn[0] += 0.0005
                    else:
                        print("boton soltado")
                ########################################  for my own joystick
                #      ^           #     ^            #
                #    ABS_Y         #    ABS_RY        #
                #  ←─────→ ABS_X #  ←─────→ ABS_RX   #
                #     ↓           #     ↓            #  
                #######################################
                elif event.type == ecodes.EV_ABS:
                    absevent = categorize(event)
                    if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":  
                        self.L3[0]=absevent.event.value-127
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                        self.L3[1]=absevent.event.value-127
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":
                        self.R3[0]=absevent.event.value-127
#                        print(self.d_z)
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":
                        self.R3[1]=absevent.event.value-127
        
        if self.poseMode == False:           
            self.V = np.sqrt(self.L3[1]**2 + self.L3[0]**2)/100.
            self.angle = np.rad2deg(np.arctan2(-self.L3[0] , -self.L3[1]))
            self.Wrot = -self.R3[0]/250.
    #        Lrot = 0.
            if self.V <= 0.035:
                self.V = 0.
            if self.Wrot <= 0.035 and self.Wrot >= -0.035:
                self.Wrot = 0.
        else:
            self.CoM_orn[0] = np.deg2rad(self.R3[0]/3)
            self.CoM_orn[1] = np.deg2rad(self.L3[1]/3)
            self.CoM_orn[2] = -np.deg2rad(self.L3[0]/3)
            self.CoM_pos[0] = -self.R3[1]/5000
            
        return self.CoM_pos , self.CoM_orn , self.V , -self.angle , -self.Wrot , self.T , self.compliantMode