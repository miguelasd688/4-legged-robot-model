#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 18:20:45 2020

@author: linux-asd
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

def solve_R(coord):
    x = coord[0,1]  #it changes the body_frame to the leg_frame
    y = -coord[0,2] #(which has different axis)
    z = -coord[0,0]
    
    """LEG DIMENTIONS"""
    coxa = 0.036
    femur = 0.11
    tibia = 0.11
    D = (x**2+y**2-coxa**2+z**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(-y,x)-numpy.arctan2(numpy.sqrt(x**2+y**2-coxa**2),-coxa)
    alpha = numpy.arctan2(z,numpy.sqrt(x**2+y**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.matrix([-tetta, alpha, gamma])
    return angles

def solve_L(coord):
    x = coord[0,1]  #it changes the body_frame to the leg_frame
    y = -coord[0,2] #(which has different axis)
    z = -coord[0,0]
    
    """LEG DIMENTIONS"""
    coxa = -0.036
    femur = 0.11
    tibia = 0.11
    D = (x**2+y**2-coxa**2+z**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(-y,x)-numpy.arctan2(numpy.sqrt(x**2+y**2-coxa**2),-coxa)
    alpha = numpy.arctan2(z,numpy.sqrt(x**2+y**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.matrix([-tetta, alpha, gamma])
    return angles
