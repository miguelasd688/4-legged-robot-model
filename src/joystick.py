#!/usr/bin/env python2
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
        self.compliantMode = False
        self.CoM_move = np.zeros(3)
    def read(self):
        r,w,x = select([self.gamepad.fd], [], [], 0.)
        
        if r:
            for event in self.gamepad.read():
#                print(event)
                if event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        if event.code == 544:#up arrow
                            self.T += 0.05
                        if event.code == 545:#down arrow
                            self.T -= 0.05
                        if event.code == 308:#square
                            if self.compliantMode == True:
                                self.compliantMode = False
                            elif self.compliantMode == False:
                                self.compliantMode = True    
                        if event.code == 310:#R1
                            self.CoM_move[0] += 0.0005
                        if event.code == 311:#L1
                            self.CoM_move[0] -= 0.0005
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
                        
        L = np.sqrt(self.L3[1]**2 + self.L3[0]**2)/250.
        angle = np.rad2deg(np.arctan2(-self.L3[0] , -self.L3[1]))
        Lrot = -self.R3[0]/250.
#        Lrot = 0.
        if L <= 0.035:
            L = 0.
        if Lrot <= 0.035 and Lrot >= -0.035:
            Lrot = 0.
            
#        pitch = np.deg2rad(self.R3[1]/2)
#        yaw = np.deg2rad(self.R3[0]/3)
        pitch = 0.
        yaw = 0.
        return self.CoM_move , L , -angle , -Lrot , self.T , self.compliantMode , yaw , pitch
