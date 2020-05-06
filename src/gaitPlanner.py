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

#gait planner in order to move all feet
class trotGait:
    def __init__(self):
        self.bodytoFeet = np.zeros([4,3])  
        self.phi = 0.
        self.lastTime = 0.
        self.alpha = 0.
        self.s = False
    
    def calculateBezier(self , phi , L , angle):#phi between [0,1), angle in degrees
    #curve generator https://www.desmos.com/calculator/xlpbe9bgll
        if (phi >= 1):
            phi = phi - 1.
        X = np.cos(np.deg2rad(angle))#cylindrical coordinates
        Y = np.sin(np.deg2rad(angle))
        
        if (phi >= 0.3 and phi < 0.8):
            self.s = True
#            print('foot DOWN' , self.s , phi)
            
        elif (phi >= 0.8 or phi < 0.3):
            self.s = False
#            print('foot UP', self.s , phi)
            
        
        X = np.array([0. ,
                     -X*L*0.1 ,
                     -X*L*0.05 , 
                     X*L*0.15 ,
                     X*L*0.05 ,
                     X*L*0.1 , 
                     0.])
        Y = np.array([0. , 
                     -Y*L*0.1 ,
                     -Y*L*0.05 , 
                     Y*L*0.15 , 
                     Y*L*0.05 , 
                     Y*L*0.1 ,
                     0.])
        Z = np.array([0. , 0. , 0. , np.abs(L)*0.08 , 0. , 0. , 0.])
        stepX = 0.
        stepY = 0.
        stepZ = 0.
        for i in range(6): #sum all terms of the curve
            stepX = stepX + b(phi,i,X[i]) 
            stepY = stepY + b(phi,i,Y[i])
            stepZ = stepZ + b(phi,i,Z[i])
            
        return stepX, stepY , stepZ , self.s


    def stepTrajectory(self , phi , L , angle , Lrot , centerToFoot): #phi belong [0,1), angles in degrees
        stepX_long , stepY_long , stepZ_long , s = self.calculateBezier(phi , L , angle)#longitudinal step
        
        r = np.sqrt(centerToFoot[0]**2 + centerToFoot[1]**2)
        footAngle = np.arctan2(centerToFoot[1],centerToFoot[0])#step describes a circuference in order to rotate
        stepX_rot , stepY_rot , stepZ_rot , s = self.calculateBezier(phi , Lrot , 90. + np.rad2deg(footAngle - self.alpha))
        if (centerToFoot[1] > 0):#define the sign for every quadrant 
            if (stepX_rot < 0):
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)   
        else:
            if (stepX_rot < 0):
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)   

        coord = np.empty(3)        
        coord[0] = stepX_long + stepX_rot
        coord[1] = stepY_long + stepY_rot
        coord[2] = stepZ_long + stepZ_rot
        
        return coord , s
        
        
    #computes step trajectory for every foot, defining L which is like velocity command, its angle, 
    #offset between each foot, period of time of each step and the initial vector from center of robot to feet.
    #defined in the foot local frame
    def loop(self , L , angle , Lrot , T , offset , bodytoFeet_ ):
        if (self.phi >= 0.99):
            self.lastTime= time.time()
        self.phi = (time.time()-self.lastTime)/T
#        print(self.phi)
        
        step_coord , sFR = self.stepTrajectory(self.phi + offset[0] , L , angle , Lrot , np.squeeze(np.asarray(bodytoFeet_[0,:]))) #FR
        self.bodytoFeet[0,0] =  bodytoFeet_[0,0] + step_coord[0]
        self.bodytoFeet[0,1] =  bodytoFeet_[0,1] + step_coord[1] 
        self.bodytoFeet[0,2] =  bodytoFeet_[0,2] + step_coord[2]
    
        step_coord , sFL = self.stepTrajectory(self.phi + offset[1] , L , angle , Lrot , np.squeeze(np.asarray(bodytoFeet_[1,:])))#FL
        self.bodytoFeet[1,0] =  bodytoFeet_[1,0] + step_coord[0]
        self.bodytoFeet[1,1] =  bodytoFeet_[1,1] + step_coord[1]
        self.bodytoFeet[1,2] =  bodytoFeet_[1,2] + step_coord[2]
        
        step_coord , sBR = self.stepTrajectory(self.phi + offset[2] , L , angle , Lrot , np.squeeze(np.asarray(bodytoFeet_[2,:])))#BR
        self.bodytoFeet[2,0] =  bodytoFeet_[2,0] + step_coord[0]
        self.bodytoFeet[2,1] =  bodytoFeet_[2,1] + step_coord[1]
        self.bodytoFeet[2,2] =  bodytoFeet_[2,2] + step_coord[2]

        step_coord , sBL = self.stepTrajectory(self.phi + offset[3] , L , angle , Lrot , np.squeeze(np.asarray(bodytoFeet_[3,:])))#BL
        self.bodytoFeet[3,0] =  bodytoFeet_[3,0] + step_coord[0]
        self.bodytoFeet[3,1] =  bodytoFeet_[3,1] + step_coord[1]
        self.bodytoFeet[3,2] =  bodytoFeet_[3,2] + step_coord[2]
            
        S = [sFR , sFL , sBR , sBL]
        return self.bodytoFeet , S
