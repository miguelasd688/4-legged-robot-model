#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 18:20:45 2020

@author: miguel-asd
"""
import numpy

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
#IK equations now written in pybullet frame.
def solve_R(coord , coxa , femur , tibia): 
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles

def solve_L(coord , coxa , femur , tibia):
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles

