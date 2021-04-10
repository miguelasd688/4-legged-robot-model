#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May  1 21:21:02 2020

@author: miguel_asd
"""
import numpy as np
import pybullet as p
import time


class systemStateEstimator:
    def __init__(self , boxId):
        self.boxId = boxId
        self.body_index = 0
        #enable contact force stimation
        for i in range(p.getNumJoints(self.boxId)):
            p.enableJointForceTorqueSensor(self.boxId, i, enableSensor = True)
        self.IdFR = p.addUserDebugLine([0,0,0] , [0,0,0] , lineColorRGB = [0,1,0] , lineWidth = 3)
        self.IdFL = p.addUserDebugLine([0,0,0] , [0,0,0] , lineColorRGB = [0,1,0] , lineWidth = 3)
        self.IdBR = p.addUserDebugLine([0,0,0] , [0,0,0] , lineColorRGB = [0,1,0] , lineWidth = 3)
        self.IdBL = p.addUserDebugLine([0,0,0] , [0,0,0] , lineColorRGB = [0,1,0] , lineWidth = 3)


        self.X = np.empty([12,1])
        self.U = np.empty([12,1])
        self.torque = np.empty([12,1])
        self.tau = np.empty([12,1])
        self.t = np.empty([1])
        self.startTime = time.time()
        
    def states(self):
        
        states = p.getLinkState(self.boxId ,self.body_index , computeLinkVelocity = True)
        worldPos = states[0]
        worldOrn = p.getEulerFromQuaternion(states[1])
        linearVel = states[6]
        angularVel = states[7]
        stateVector = np.concatenate((worldOrn , worldPos , angularVel , linearVel)).reshape(12,1)
        
        timeNow = time.time() - self.startTime
        self.t = np.append(self.t , timeNow)
        self.X = np.append(self.X , stateVector , axis = 1)
        return self.t , self.X
        
    def controls(self):
        footFR_index = 3
        footFL_index = 7
        footBR_index = 11
        footBL_index = 15   
        index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        jointInfo = p.getJointStates(self.boxId ,index)
#        print(jointInfo)
        footPosFR = p.getLinkState(self.boxId ,footFR_index)
        footPosFL = p.getLinkState(self.boxId ,footFL_index)
        footPosBR = p.getLinkState(self.boxId ,footBR_index)
        footPosBL = p.getLinkState(self.boxId ,footBL_index)
        forceFR = np.array(jointInfo[3][2][0:3])
        forceFL = np.array(jointInfo[7][2][0:3])
        forceBR = np.array(jointInfo[11][2][0:3])
        forceBL = np.array(jointInfo[15][2][0:3])
        torqueFR = np.array([[jointInfo[0][3]],[jointInfo[1][3]],[jointInfo[2][3]]])
        torqueFL = np.array([[jointInfo[4][3]],[jointInfo[5][3]],[jointInfo[6][3]]])
        torqueBR = np.array([[jointInfo[8][3]],[jointInfo[9][3]],[jointInfo[10][3]]])
        torqueBL = np.array([[jointInfo[12][3]],[jointInfo[13][3]],[jointInfo[14][3]]])
        
        for i in range(3):
            if (forceFR[i] > 0.):
                forceFR[i] = 0.
            if (forceFL[i] > 0.):
                forceFL[i] = 0.
            if (forceBR[i] > 0.):
                forceBR[i] = 0.
            if (forceBL[i] > 0.):
                forceBL[i] = 0.
                
        contactForces = np.transpose(np.matrix([forceFR[0],forceFR[1],forceFR[2],
                                         forceFL[0],forceFL[1],forceFL[2],
                                         forceBR[0],forceBR[1],forceBR[2],
                                         forceBL[0],forceBL[1],forceBL[2]]))
        torquei = np.concatenate((torqueFR,torqueFL,torqueBR,torqueBL))
        #draw force vector
#        p.addUserDebugLine(footPosFR[4], footPosFR[4] - forceFR/100, lineColorRGB = [0,1,0] , lineWidth = 3 , replaceItemUniqueId = self.IdFR)
#        p.addUserDebugLine(footPosFL[4], footPosFL[4] - forceFL/100, lineColorRGB = [0,1,0] , lineWidth = 3 , replaceItemUniqueId = self.IdFL)
#        p.addUserDebugLine(footPosBR[4], footPosBR[4] - forceBR/100, lineColorRGB = [0,1,0] , lineWidth = 3 , replaceItemUniqueId = self.IdBR)
#        p.addUserDebugLine(footPosBL[4], footPosBL[4] - forceBL/100, lineColorRGB = [0,1,0] , lineWidth = 3 , replaceItemUniqueId = self.IdBL)
        
        self.torque = np.append(self.torque, torquei , axis = 1)
        self.U = np.append(self.U , contactForces , axis = 1)
        return self.U , contactForces , self.torque
        
    
    
    def getMotorJointStates(self , boxId):
        joint_states = p.getJointStates(boxId, range(p.getNumJoints(boxId)))
        joint_infos = [p.getJointInfo(boxId, i) for i in range(p.getNumJoints(boxId))]
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques
    
    
    
    
    
    
    
    
    def footJacobian(self , boxId , foot_index):
    
        mpos, mvel, mtorq = self.getMotorJointStates(boxId)
        result = p.getLinkState(boxId,
                            foot_index,
                            computeLinkVelocity=1,
                            computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        zero_vec = [0.0] * len(mpos)
        #3x18 (6 body dof, 3dof per leg)
        linearJ , rotJ = p.calculateJacobian(boxId , foot_index , localPosition=com_trn , 
                                          objPositions=mpos , objVelocities=zero_vec , objAccelerations=zero_vec)
        
        
        return linearJ
#    
        
    def groundForceControl(self , boxId , orn , forceVector):
        """compute torque for each joint from contact force (calculated on MPC)"""

        Ri = np.empty([3,3])
        
        R4 = Rxyz(orn[0],orn[1],orn[2])
        for i in range(3):
            for j in range(3):
                Ri[i,j] = R4[i,j]
        Rf1 = np.concatenate((Ri,np.zeros((3,3)),np.zeros((3,3)),np.zeros((3,3))),axis=1)
        Rf2 = np.concatenate((np.zeros((3,3)),Ri,np.zeros((3,3)),np.zeros((3,3))),axis=1)
        Rf3 = np.concatenate((np.zeros((3,3)),np.zeros((3,3)),Ri,np.zeros((3,3))),axis=1)
        Rf4 = np.concatenate((np.zeros((3,3)),np.zeros((3,3)),np.zeros((3,3)),Ri),axis=1)
        R = np.concatenate((Rf1,Rf2,Rf3,Rf4),axis=0)
    
        Ji = np.zeros([12,12])
        for k in range(4):
            foot_index = 3 + 4*k
            linearJ = self.footJacobian(boxId , foot_index)
            for i in range(3):
                for j in range(3):
                    Ji[i+3*k,j+3*k] = linearJ[i][6+j+3*k]
          
        tau_i = np.dot(np.dot(np.transpose(Ji),np.transpose(R)),forceVector)
        self.tau = np.append(self.tau,tau_i,axis = 1)
#        print(tau)
        return self.tau
        
        
        
        
        
        
        
        
        
        
        
        
        