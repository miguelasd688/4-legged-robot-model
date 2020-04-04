#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 27 15:21:52 2020

@author: linux-asd
"""
import numpy as np
import IK_solver as IK
import geometrics as geo


    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
"""
"using pybullet frame"
"  z                     "
"    |                   "
"    |                   "
"    |    /  y           "
"    |   /               "
"    |  /                "
"    | /                 "
"    |/_____________  x       "
"""
class robotKinematics:
    def __init__(self):
        self.targetAngs = np.matrix([0 , np.pi/4 , -np.pi/2, 0 ,#BR
                                     0 , np.pi/4 , -np.pi/2, 0 ,#BL
                                     0 , np.pi/4 , -np.pi/2, 0 ,#FL
                                     0 , np.pi/4 , -np.pi/2, 0 ])#FR
                
        """in meter """
        self.L = 0.192 #length of robot joints
        self.W = 0.075 #width of robot joints
        self.coxa = 0.04#coxa length
        self.femur = 0.10#femur length
        self.tibia = 0.10#tibia length
        """initial foot position"""
        #foot separation (0.182 -> tetta=0) and distance to floor
        self.Ydist = 0.11
        self.Xdist = self.L
        self.height = 0.15
        #body frame to coxa frame vector
        self.bodytoFR0 = np.array([ self.L/2, -self.W/2 , 0])
        self.bodytoFL0 = np.array([ self.L/2,  self.W/2 , 0])
        self.bodytoBR0 = np.array([-self.L/2, -self.W/2 , 0])
        self.bodytoBL0 = np.array([-self.L/2,  self.W/2 , 0])
        #body frame to foot frame vector
        self.bodytoFR4 = np.array([ self.Xdist/2 , -self.Ydist/2 , -self.height])
        self.bodytoFL4 = np.array([ self.Xdist/2 ,  self.Ydist/2 , -self.height])
        self.bodytoBR4 = np.array([-self.Xdist/2 , -self.Ydist/2 , -self.height])
        self.bodytoBL4 = np.array([-self.Xdist/2 ,  self.Ydist/2 , -self.height])

    def solve(self, orn , pos , bodytoFeet):
        bodytoFR4 = np.array([bodytoFeet[0,0],bodytoFeet[0,1],bodytoFeet[0,2]])
        bodytoFL4 = np.array([bodytoFeet[1,0],bodytoFeet[1,1],bodytoFeet[1,2]])
        bodytoBR4 = np.array([bodytoFeet[2,0],bodytoFeet[2,1],bodytoFeet[2,2]])
        bodytoBL4 = np.array([bodytoFeet[3,0],bodytoFeet[3,1],bodytoFeet[3,2]])

        """defines 4 vertices which rotates with the body"""
        _bodytoFR0 = geo.transform(self.bodytoFR0 , orn, pos)
        _bodytoFL0 = geo.transform(self.bodytoFL0 , orn, pos)
        _bodytoBR0 = geo.transform(self.bodytoBR0 , orn, pos)
        _bodytoBL0 = geo.transform(self.bodytoBL0 , orn, pos)
        """defines coxa_frame to foot_frame leg vector neccesary for IK"""
        FRcoord = bodytoFR4 - _bodytoFR0
        FLcoord = bodytoFL4 - _bodytoFL0
        BRcoord = bodytoBR4 - _bodytoBR0
        BLcoord = bodytoBL4 - _bodytoBL0
        """undo transformation of leg vector to keep feet still"""
        undoOrn = -orn
        undoPos = -pos
        _FRcoord = geo.transform(FRcoord , undoOrn, undoPos)
        _FLcoord = geo.transform(FLcoord , undoOrn, undoPos)
        _BRcoord = geo.transform(BRcoord , undoOrn, undoPos)
        _BLcoord = geo.transform(BLcoord , undoOrn, undoPos)
    
        FR_angles = IK.solve_R(_FRcoord , self.coxa , self.femur , self.tibia)
        FL_angles = IK.solve_L(_FLcoord , self.coxa , self.femur , self.tibia)
        BR_angles = IK.solve_R(_BRcoord , self.coxa , self.femur , self.tibia)
        BL_angles = IK.solve_L(_BLcoord , self.coxa , self.femur , self.tibia)
        
        return FR_angles, FL_angles, BR_angles, BL_angles
