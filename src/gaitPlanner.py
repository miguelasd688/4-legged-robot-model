#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 16:38:15 2020

@author: linux-asd
"""
import time
import numpy as np

#function neccesary to build a parametrized bezier curve 
def f(n,k): #calculates binomial factor (n k)
    return np.math.factorial(n)/(np.math.factorial(k)*np.math.factorial(n-k))

def b(t,k,point):
    n = 6 #7 points bezier curve
    return point*f(n,k)*np.power(t,k)*np.power(1-t,n-k)

def calculateBezier(phi , L):#phi between [0,1)
    #you can generate bezier curve in here https://www.desmos.com/calculator/xlpbe9bgll
    x = np.array([L*0. , -L*0.1 , -L*0.05 , L*0.15 , L*0.05 , L*0.1 , 0.])
    y = np.array([0. , 0. , 0. , L*0.1 , 0. , 0. , 0.])
    stepX = 0.
    stepY = 0.
    for i in range(6): #sum all terms of the curve
        stepX = stepX + b(phi,i,x[i]) 
        stepY = stepY + b(phi,i,y[i])
    return stepX , stepY


def stepTrajectory(phi , L , angle): #phi belong [0,1)
    if (phi >= 1):
        phi = phi - 1
    coord = np.empty(3)
    stepX , stepY = calculateBezier(phi,L)
    coord[0] = np.cos(np.deg2rad(angle))*stepX              
    coord[1] = np.sin(np.deg2rad(angle))*stepX
    coord[2] = stepY
    
    return coord

#gait planner in order to move all feet
class trotGait:
    def __init__(self):
        self.bodytoFeet = np.empty([4,3])  
        self.phi = 0.
        self.lastTime = 0.
    #computes step trajectory for every foot, defining L which is step length, its angle, offset between each foot, 
    #period of time of each step and the initial vector from center of robot to feet.
    def loop(self , L , angle , T , offset , bodytoFeet_ ):
        if (self.phi >= 1):
            self.lastTime= time.time()
        self.phi = (time.time()-self.lastTime)/T
        step_coord = stepTrajectory(self.phi + offset[0] , L , angle) #FR
        self.bodytoFeet[0,0] =  bodytoFeet_[0,0] + step_coord[0]
        self.bodytoFeet[0,1] =  bodytoFeet_[0,1] - step_coord[1] 
        self.bodytoFeet[0,2] =  bodytoFeet_[0,2] + step_coord[2]
    
        step_coord = stepTrajectory(self.phi + offset[1] , L , angle)#FL
        self.bodytoFeet[1,0] =  bodytoFeet_[1,0] + step_coord[0]
        self.bodytoFeet[1,1] =  bodytoFeet_[1,1] - step_coord[1]
        self.bodytoFeet[1,2] =  bodytoFeet_[1,2] + step_coord[2]
        
        step_coord = stepTrajectory(self.phi + offset[2] , L , angle)#BR
        self.bodytoFeet[2,0] =  bodytoFeet_[2,0] + step_coord[0]
        self.bodytoFeet[2,1] =  bodytoFeet_[2,1] + step_coord[1]
        self.bodytoFeet[2,2] =  bodytoFeet_[2,2] + step_coord[2]

        step_coord = stepTrajectory(self.phi + offset[3] , L , angle)#BL
        self.bodytoFeet[3,0] =  bodytoFeet_[3,0] + step_coord[0]
        self.bodytoFeet[3,1] =  bodytoFeet_[3,1] + step_coord[1]
        self.bodytoFeet[3,2] =  bodytoFeet_[3,2] + step_coord[2]
            
        return self.bodytoFeet
