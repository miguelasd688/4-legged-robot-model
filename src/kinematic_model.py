#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 27 15:21:52 2020

@author: mmiguel-asd
"""
import numpy as np
from . import IK_solver 
from . import geometrics


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
        self.targetAngs = np.matrix([0 , 0 , -0, 0 ,#BR
                                     0 , 0 , -0, 0 ,#BL
                                     0 , 0 , -0, 0 ,#FL
                                     0 , 0 , -0, 0 ])#FR
                
        """in meter """
        self.L = .18 #length of robot joints
        self.W = 0.064 #width of robot joints
        self.coxa = 0.0375 #coxa length
        self.femur = 0.08 #femur length
        self.tibia = 0.08 #tibia length
        """initial foot position"""
        #foot separation (0.182 -> tetta=0) and distance to floor
        self.Ydist = 11.
        self.Xdist = self.L
        self.height = 15.
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
        bodytoFR4 = np.asarray([bodytoFeet[0,0],bodytoFeet[0,1],bodytoFeet[0,2]])
        bodytoFL4 = np.asarray([bodytoFeet[1,0],bodytoFeet[1,1],bodytoFeet[1,2]])
        bodytoBR4 = np.asarray([bodytoFeet[2,0],bodytoFeet[2,1],bodytoFeet[2,2]])
        bodytoBL4 = np.asarray([bodytoFeet[3,0],bodytoFeet[3,1],bodytoFeet[3,2]])

        """defines 4 vertices which rotates with the body"""
        _bodytoFR0 = geometrics.transform(self.bodytoFR0 , orn, pos)
        _bodytoFL0 = geometrics.transform(self.bodytoFL0 , orn, pos)
        _bodytoBR0 = geometrics.transform(self.bodytoBR0 , orn, pos)
        _bodytoBL0 = geometrics.transform(self.bodytoBL0 , orn, pos)
        """defines coxa_frame to foot_frame leg vector neccesary for IK"""
        FRcoord = bodytoFR4 - _bodytoFR0
        FLcoord = bodytoFL4 - _bodytoFL0
        BRcoord = bodytoBR4 - _bodytoBR0
        BLcoord = bodytoBL4 - _bodytoBL0
        """undo transformation of leg vector to keep feet still"""
        undoOrn = -orn
        undoPos = -pos
        _FRcoord = geometrics.transform(FRcoord , undoOrn, undoPos)
        _FLcoord = geometrics.transform(FLcoord , undoOrn, undoPos)
        _BRcoord = geometrics.transform(BRcoord , undoOrn, undoPos)
        _BLcoord = geometrics.transform(BLcoord , undoOrn, undoPos)
        
#        print(_FRcoord,_BRcoord)
        FR_angles = IK_solver.solve_FR(_FRcoord , self.coxa , self.femur , self.tibia)
        FL_angles = IK_solver.solve_FL(_FLcoord , self.coxa , self.femur , self.tibia)
        BR_angles = IK_solver.solve_BR(_BRcoord , self.coxa , self.femur , self.tibia)
        BL_angles = IK_solver.solve_BL(_BLcoord , self.coxa , self.femur , self.tibia)

        
        _bodytofeetFR = _bodytoFR0 + _FRcoord
        _bodytofeetFL = _bodytoFL0 + _FLcoord
        _bodytofeetBR = _bodytoBR0 + _BRcoord
        _bodytofeetBL = _bodytoBL0 + _BLcoord
        _bodytofeet = np.matrix([[_bodytofeetFR[0] , _bodytofeetFR[1] , _bodytofeetFR[2]],
                                 [_bodytofeetFL[0] , _bodytofeetFL[1] , _bodytofeetFL[2]],
                                 [_bodytofeetBR[0] , _bodytofeetBR[1] , _bodytofeetBR[2]],
                                 [_bodytofeetBL[0] , _bodytofeetBL[1] , _bodytofeetBL[2]]])
        
        return FR_angles, FL_angles, BR_angles, BL_angles , _bodytofeet
