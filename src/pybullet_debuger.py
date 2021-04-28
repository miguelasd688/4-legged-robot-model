#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 22:15:21 2020

@author: miguel-asd
"""
import pybullet as p
import time
import numpy as np
import sys

class pybulletDebug:
    def __init__(self , mode = "STATES"):
        #Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
        self.cyaw=90
        self.cpitch=-7
        self.cdist=0.30
        time.sleep(0.5)
        
        self.mode = mode
        if mode == "STATES":
            self.xId = p.addUserDebugParameter("x" , -0.02 , 0.02 , 0.)
            self.yId = p.addUserDebugParameter("y" , -0.02 , 0.02 , 0.)
            self.zId = p.addUserDebugParameter("z" , -0.02 , 0.02 , 0.)
            self.rollId = p.addUserDebugParameter("roll" , -np.pi/4 , np.pi/4 , 0.)
            self.pitchId = p.addUserDebugParameter("pitch" , -np.pi/4 , np.pi/4 , 0.)
            self.yawId = p.addUserDebugParameter("yaw" , -np.pi/4 , np.pi/4 , 0.)
            self.LId = p.addUserDebugParameter("L" , -0.50 , 1 , 0.)
            self.LrotId = p.addUserDebugParameter("Lrot" , -1.50 , 1.50 , 0.)
            self.angleId = p.addUserDebugParameter("angleWalk" , -180. , 180. , 0.)
            self.periodId = p.addUserDebugParameter("stepPeriod" , 0.1 , 3. , 1.0)
            self.step_dur_asym = p.addUserDebugParameter("step_dur_asym" , -2 , 2. , 0.0)
            
        elif mode == "CALIBRATION":
            self.FR_coxaId = p.addUserDebugParameter("FR_coxa" , -90. , 90. , 0.)
            self.FR_femurId = p.addUserDebugParameter("FR_femur" , -90. , 90. , 0.)
            self.FR_tibiaId = p.addUserDebugParameter("FR_tibia" , -90. , 90. , 0.)
            
            self.FL_coxaId = p.addUserDebugParameter("FL_coxa" , -90. , 90. , 0.)
            self.FL_femurId = p.addUserDebugParameter("FL_femur" , -90. , 90. , 0.)
            self.FL_tibiaId = p.addUserDebugParameter("FL_tibia" , -90. , 90. , 0.)
            
            self.BR_coxaId = p.addUserDebugParameter("BR_coxa" , -90. , 90. , 0.)
            self.BR_femurId = p.addUserDebugParameter("BR_femur" , -90. , 90. , 0.)
            self.BR_tibiaId = p.addUserDebugParameter("BR_tibia" , -90. , 90. , 0.)
            
            self.BL_coxaId = p.addUserDebugParameter("BL_coxa" , -90. , 90. , 0.)
            self.BL_femurId = p.addUserDebugParameter("BL_femur" , -90. , 90. , 0.)
            self.BL_tibiaId = p.addUserDebugParameter("BL_tibia" , -90. , 90. , 0.)
            
    
    def cam_and_robotstates(self , boxId):
        
        if self.mode != "DIRECT":
                    ####orientacion de la camara
            cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
            p.resetDebugVisualizerCamera( cameraDistance=self.cdist, cameraYaw=self.cyaw, cameraPitch=self.cpitch, cameraTargetPosition=cubePos)
            keys = p.getKeyboardEvents()
            #Keys to change camera
            if keys.get(100):  #D
                self.cyaw+=1
            if keys.get(97):   #A
                self.cyaw-=1
            if keys.get(99):   #C
                self.cpitch+=1
            if keys.get(102):  #F
                self.cpitch-=1
            if keys.get(122):  #Z
                self.cdist+=0.01
            if keys.get(120):  #X
                self.cdist-=0.01
            if keys.get(27):  #ESC
                p.disconnect()
                time.sleep(2)    
#               sys.exit()
        else:
            debugOutput = 0.

        if self.mode == "STATES":
            #read position from debug
            pos = np.array([p.readUserDebugParameter(self.xId),p.readUserDebugParameter(self.yId), p.readUserDebugParameter(self.zId)])
            orn = np.array([p.readUserDebugParameter(self.rollId),p.readUserDebugParameter(self.pitchId), p.readUserDebugParameter(self.yawId)])
            L = p.readUserDebugParameter(self.LId)
            Lrot = p.readUserDebugParameter(self.LrotId)
            angle = p.readUserDebugParameter(self.angleId)
            T = p.readUserDebugParameter(self.periodId)
            sda =  p.readUserDebugParameter(self.step_dur_asym)
                
            debugOutput = pos , orn , L , Lrot , angle , T , sda
                
        elif self.mode == "CALIBRATION":
            FR_coxa_angle = np.deg2rad(p.readUserDebugParameter(self.FR_coxaId))
            FR_femur_angle = np.deg2rad(p.readUserDebugParameter(self.FR_femurId)) 
            FR_tibia_angle = np.deg2rad(p.readUserDebugParameter(self.FR_tibiaId))
            
            FL_coxa_angle = np.deg2rad(p.readUserDebugParameter(self.FL_coxaId))
            FL_femur_angle = np.deg2rad(p.readUserDebugParameter(self.FL_femurId)) 
            FL_tibia_angle = np.deg2rad(p.readUserDebugParameter(self.FL_tibiaId))
            
            BR_coxa_angle = np.deg2rad(p.readUserDebugParameter(self.BR_coxaId))
            BR_femur_angle = np.deg2rad(p.readUserDebugParameter(self.BR_femurId))
            BR_tibia_angle = np.deg2rad(p.readUserDebugParameter(self.BR_tibiaId))
            
            BL_coxa_angle = np.deg2rad(p.readUserDebugParameter(self.BL_coxaId))
            BL_femur_angle = np.deg2rad(p.readUserDebugParameter(self.BL_femurId))
            BL_tibia_angle = np.deg2rad(p.readUserDebugParameter(self.BL_tibiaId))
            
            debugOutput = np.array([[FR_coxa_angle , FR_femur_angle , FR_tibia_angle],
                                    [FL_coxa_angle , FL_femur_angle , FL_tibia_angle],
                                    [BR_coxa_angle , BR_femur_angle , BR_tibia_angle],
                                    [BL_coxa_angle , BL_femur_angle , BL_tibia_angle]])
    
        return debugOutput
