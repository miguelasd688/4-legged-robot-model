#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 18:20:45 2020

@author: miguel-asd
"""
import numpy as np

def checkdomain(D):
    if D > 1 or D < -1:
        print("____OUT OF DOMAIN____")
        if D > 1: 
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
    else:
        return D
#this is based on this paper: 
#"https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot"
"""
"using pybullet frame"
"  z                     "
"    |                   "
"    |                   "
"    |    /  y           "
"    |   /               "
"    |  /                "
"    | /                 "
"    |/____________  x       "
"""

#targetPos = [0 , np.pi/4. , -np.pi/2., 0 ,#BR
#             0 , np.pi/4. , -np.pi/2., 0 ,#BL
#             0 , -np.pi/4. , np.pi/2., 0 ,#FL
#             0 , -np.pi/4. , np.pi/2., 0 ]#FR
#IK equations written in pybullet frame.
def IK(coord , coxa , femur , tibia , sign): 
    D = (coord[1]**2+coord[2]**2-coxa**2+coord[0]**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = sign*np.arctan2(np.sqrt(1-D**2),D)
    tetta = -np.arctan2(-coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+coord[2]**2-coxa**2),-coxa)
    alpha = np.arctan2(coord[0],np.sqrt(coord[1]**2+coord[2]**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
    angles = np.array([tetta, alpha, gamma])
    return angles


def legs_IK(FRcoord , FLcoord , BRcoord , BLcoord , coxa , femur , tibia , kneeConfig = "<<"):
    if kneeConfig == "<<":
        signFront = 1.
        signBack = 1.
    elif kneeConfig == "><":
        signFront = 1.
        signBack = -1.
    
    FR_angles = IK(FRcoord , coxa , femur , tibia , signFront)
    FL_angles = IK(FLcoord , -coxa , femur , tibia , signFront)
    BR_angles = IK(BRcoord , coxa , femur , tibia , signBack)
    BL_angles = IK(BLcoord , -coxa , femur , tibia , signBack)
    
    angles = np.matrix([[FR_angles[0],FR_angles[1]+np.pi/4.,FR_angles[2]-np.pi/2.],
                        [FL_angles[0],FL_angles[1]+np.pi/4.,FL_angles[2]-np.pi/2.],
                        [BR_angles[0],BR_angles[1]-np.pi/4.,BR_angles[2]+np.pi/2.],
                        [BL_angles[0],BL_angles[1]-np.pi/4.,BL_angles[2]+np.pi/2.]])
    
    return angles
