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
def solve_FR(coord , coxa , femur , tibia): 
    D = (coord[1]**2+coord[2]**2-coxa**2+coord[0]**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = np.arctan2(np.sqrt(1-D**2),D)
    tetta = -np.arctan2(-coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+coord[2]**2-coxa**2),-coxa)
    alpha = np.arctan2(coord[0],np.sqrt(coord[1]**2+coord[2]**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
    angles = np.array([tetta, alpha+np.pi/4., gamma-np.pi/2.])
    return angles

def solve_FL(coord , coxa , femur , tibia):
    D = (coord[1]**2+(-coord[2])**2-coxa**2+coord[0]**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = np.arctan2(np.sqrt(1-D**2),D)
    tetta = -np.arctan2(-coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),coxa)
    alpha = np.arctan2(coord[0],np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
    angles = np.array([tetta, alpha+np.pi/4., gamma-np.pi/2.])
    return angles

def solve_BR(coord , coxa , femur , tibia): 
    D = (coord[1]**2+coord[2]**2-coxa**2+coord[0]**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = np.arctan2(-np.sqrt(1-D**2),D)
    tetta = -np.arctan2(-coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+coord[2]**2-coxa**2),-coxa)
    alpha = np.arctan2(coord[0],np.sqrt(coord[1]**2+coord[2]**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
    angles = np.array([tetta, alpha-np.pi/4., gamma+np.pi/2.])
    return angles

def solve_BL(coord , coxa , femur , tibia):
    D = (coord[1]**2+coord[2]**2-coxa**2+coord[0]**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = np.arctan2(-np.sqrt(1-D**2),D)
    tetta = -np.arctan2(-coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+coord[2]**2-coxa**2),coxa)
    alpha = np.arctan2(coord[0],np.sqrt(coord[1]**2+coord[2]**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
    angles = np.array([tetta, alpha-np.pi/4., gamma+np.pi/2.])
    return angles

    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles

