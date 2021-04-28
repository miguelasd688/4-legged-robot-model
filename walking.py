#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: miguel-asd
"""

import time
import numpy as np
import pybullet as p
import pybullet_data
#import matplotlib.pyplot as plt
import csv

from src.kinematic_model import robotKinematics
from src.pybullet_debuger import pybulletDebug  
from src.gaitPlanner import trotGait
from src.sim_fb import systemStateEstimator


def rendering(render):
    """Enable/disable rendering"""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, render)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, render)

def robot_init( dt, body_pos, fixed = True , connect = p.GUI):
    physicsClient = p.connect(connect)#p.GUI or p.DIRECT for non-graphical version
    # turn off rendering while loading the models
    rendering(0)

    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(0)
    p.setPhysicsEngineParameter(
        fixedTimeStep=dt,
        numSolverIterations=100,
        enableFileCaching=0,
        numSubSteps=1,
        solverResidualThreshold=1e-10,
        erp=1e-1,
        contactERP=0.01,
        frictionERP=0.01,
    )
    # add floor
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.loadURDF("plane.urdf")
    # add robot
    body_id = p.loadURDF("4leggedRobot.urdf", body_pos, useFixedBase=fixed)
    joint_ids = []
    
    #robot properties

    maxVel = 3.703 #rad/s
    for j in range(p.getNumJoints(body_id)):
        p.changeDynamics( body_id, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
        p.changeDynamics( body_id, j, maxJointVelocity=maxVel)
        joint_ids.append( p.getJointInfo(body_id, j))
#        info = p.getJointInfo( body_id, j )
#        joint_name = info[1].decode('UTF-8')
#        link_name = info[12].decode('UTF-8')
#        print(joint_name,link_name)

    # start record video
    p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot.mp4")
    rendering(1)
    
    


    
    return body_id, joint_ids



def move_joints(body_id , angles):
    maxForce = 2 #N/m
    #move movable joints
    for i in range(3):
        p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, 
                                targetPosition = angles[0,i] , force = maxForce)
        p.setJointMotorControl2(body_id, 4 + i, p.POSITION_CONTROL, 
                                targetPosition = angles[1,i] , force = maxForce)
        p.setJointMotorControl2(body_id, 8 + i, p.POSITION_CONTROL, 
                                targetPosition = angles[2,i] , force = maxForce) 
        p.setJointMotorControl2(body_id, 12 + i, p.POSITION_CONTROL, 
                                targetPosition = angles[3,i] , force = maxForce)
        
        
        
        
def robot_stepsim( body_id, body_pos, body_orn, body2feet ):
    #robot properties

    fr_index, fl_index, br_index, bl_index = 3, 7, 11, 15
    
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    angles , body2feet_ = robotKinematics.solve( body_orn , body_pos , body2feet )
    move_joints(body_id , angles)
    
    return body2feet_



def robot_quit():
    p.disconnect()





##This part of code is just to save the raw telemetry data.
fieldnames = ["t","FR","FL","BR","BL"]
with open('telemetry/data.csv','w') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames = fieldnames)
    csv_writer.writeheader()

def update_data():
    #take meassurement from simulation
    t , X = meassure.states()
    U , Ui ,torque = meassure.controls()
    
    with open('telemetry/data.csv','a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
        info = {"t" : t[-1],
                "FR" : Ui[2,0],
                "FL" : Ui[5,0],
                "BR" : Ui[8,0],
                "BL" : Ui[11,0]}
        csv_writer.writerow(info)
        
        
        
        
        
        
        
if __name__ == '__main__':
    dT = 0.002
    debugMode = "STATES"
    kneeConfig = "><"
    robotKinematics = robotKinematics(kneeConfig) # "><" or "<<"
    trot = trotGait()
    
    bodyId, jointIds = robot_init( dt = dT, body_pos = [0,0,0.2], fixed = True , connect = p.GUI)
    pybulletDebug = pybulletDebug(debugMode)
    meassure = systemStateEstimator(bodyId)

    """initial foot position"""
    #foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
    Xdist, Ydist, height = 0.18, 0.15, 0.10
    #body frame to foot frame vector
    bodytoFeet0 = np.matrix([[ Xdist/2. , -Ydist/2. , height],
                            [ Xdist/2. ,  Ydist/2. , height],
                            [-Xdist/2. , -Ydist/2. , height],
                            [-Xdist/2. ,  Ydist/2. , height]])

    offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
    footFR_index, footFL_index, footBR_index, footBL_index = 3, 7, 11, 15
    T = 0.5 #period of time (in seconds) of every step
 
    N_steps=5000000
    N_par = 8
    out = np.zeros((N_par+1,N_steps))

    # start record video
    #p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot.mp4")
    for k_ in range(0,N_steps):
        lastTime = time.time()
        if debugMode == "STATES": 
            pos , orn , L , Lrot , angle , T , sda = pybulletDebug.cam_and_robotstates(bodyId)
            
            bodytoFeet = trot.loop( L , angle , Lrot , T , offset , bodytoFeet0 , sda)
            robot_stepsim( bodyId, pos, orn, bodytoFeet )
            
            update_data()
            
        elif debugMode == "CALIBRATION":
            #No kinematic model needed for calibration
            angles = pybulletDebug.cam_and_robotstates(bodyId)
            move_joints(bodyId , angles)
            time.sleep(0.01) #slow down the simulation the hard way
        
        # out[0,k_]=k_*dT
        # out[1:4,k_]=bodytoFeet[0]
        # out[4:7,k_]=bodytoFeet[1]
        # out[7,k_]=L
        # out[8,k_]=Lrot
        # #import IPython;IPython.embed()
        p.stepSimulation()
#        print(time.time() - lastTime)
    robot_quit()




    # add robot
    body_id = p.loadURDF("4leggedRobot.urdf", body_pos, useFixedBase=fixed)
    joint_ids = []
    
    #robot properties

    maxVel = 3.703 #rad/s
    for j in range(p.getNumJoints(body_id)):
        p.changeDynamics( body_id, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
        p.changeDynamics( body_id, j, maxJointVelocity=maxVel)
        joint_ids.append( p.getJointInfo(body_id, j))
#        info = p.getJointInfo( body_id, j )
#        joint_name = info[1].decode('UTF-8')
#        link_name = info[12].decode('UTF-8')
#        print(joint_name,link_name)

    # start record video
    p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot.mp4")
    rendering(1)
    return body_id, joint_ids

def robot_stepsim( body_id, body_pos, body_orn, body2feet ):
    #robot properties

    
    fr_index, fl_index, br_index, bl_index = 3, 7, 11, 15
    maxForce = 2 #N/m
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    fr_angles, fl_angles, br_angles, bl_angles , body2feet_ = robotKinematics.solve( body_orn , body_pos , body2feet )
    #move movable joints
    for i in range(3):
        p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, 
                                targetPosition = fr_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 4 + i, p.POSITION_CONTROL, 
                                targetPosition = fl_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 8 + i, p.POSITION_CONTROL, 
                                targetPosition = br_angles[i] , force = maxForce) 
        p.setJointMotorControl2(body_id, 12 + i, p.POSITION_CONTROL, 
                                targetPosition = bl_angles[i] , force = maxForce)

    p.stepSimulation()
    
    return body2feet_

def robot_quit():
    p.disconnect()


##This part of code is just to save the raw telemetry data.
fieldnames = ["t","FR","FL","BR","BL"]
with open('telemetry/data.csv','w') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames = fieldnames)
    csv_writer.writeheader()

def update_data():
    #take meassurement from simulation
    t , X = meassure.states()
    U , Ui ,torque = meassure.controls()
    
    with open('telemetry/data.csv','a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
        info = {"t" : t[-1],
                "FR" : Ui[2,0],
                "FL" : Ui[5,0],
                "BR" : Ui[8,0],
                "BL" : Ui[11,0]}
        csv_writer.writerow(info)
        
        
        
        
        
        
        
if __name__ == '__main__':
    dT = 0.005
    bodyId, jointIds = robot_init( dt = dT, body_pos = [0,0,0.18], fixed = False )
    pybulletDebug = pybulletDebug()
    robotKinematics = robotKinematics()
    trot = trotGait()
    meassure = systemStateEstimator(bodyId)

    """initial foot position"""
    #foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
    Xdist, Ydist, height = 0.18, 0.15, 0.10
    #body frame to foot frame vector
    bodytoFeet0 = np.matrix([[ Xdist/2. , -Ydist/2. , height],
                            [ Xdist/2. ,  Ydist/2. , height],
                            [-Xdist/2. , -Ydist/2. , height],
                            [-Xdist/2. ,  Ydist/2. , height]])

    offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
    footFR_index, footFL_index, footBR_index, footBL_index = 3, 7, 11, 15
    T = 0.5 #period of time (in seconds) of every step
 
    N_steps=50000
    N_par = 8
    out = np.zeros((N_par+1,N_steps))

    # start record video
    #p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot.mp4")
    for k_ in range(0,N_steps):
        lastTime = time.time()
        pos , orn , L , angle , Lrot , T , sda= pybulletDebug.cam_and_robotstates(bodyId)
        #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
        bodytoFeet = trot.loop( L , angle , Lrot , T , offset , bodytoFeet0 , sda)
        # out[0,k_]=k_*dT
        # out[1:4,k_]=bodytoFeet[0]
        # out[4:7,k_]=bodytoFeet[1]
        # out[7,k_]=L
        # out[8,k_]=Lrot
        # #import IPython;IPython.embed()
        robot_stepsim( bodyId, pos, orn, bodytoFeet )
        update_data()
        print(time.time() - lastTime)
    robot_quit()



    # add robot
    body_id = p.loadURDF("4leggedRobot.urdf", body_pos, useFixedBase=fixed)
    joint_ids = []
    
    #robot properties

    maxVel = 3.703 #rad/s
    for j in range(p.getNumJoints(body_id)):
        p.changeDynamics( body_id, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
        p.changeDynamics( body_id, j, maxJointVelocity=maxVel)
        joint_ids.append( p.getJointInfo(body_id, j))
#        info = p.getJointInfo( body_id, j )
#        joint_name = info[1].decode('UTF-8')
#        link_name = info[12].decode('UTF-8')
#        print(joint_name,link_name)

    # start record video
    p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot.mp4")
    rendering(1)
    return body_id, joint_ids

def robot_stepsim( body_id, body_pos, body_orn, body2feet ):
    #robot properties

    
    fr_index, fl_index, br_index, bl_index = 3, 7, 11, 15
    maxForce = 2 #N/m
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    fr_angles, fl_angles, br_angles, bl_angles , body2feet_ = robotKinematics.solve( body_orn , body_pos , body2feet )
    #move movable joints
    for i in range(3):
        p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, 
                                targetPosition = fr_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 4 + i, p.POSITION_CONTROL, 
                                targetPosition = fl_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 8 + i, p.POSITION_CONTROL, 
                                targetPosition = br_angles[i] , force = maxForce) 
        p.setJointMotorControl2(body_id, 12 + i, p.POSITION_CONTROL, 
                                targetPosition = bl_angles[i] , force = maxForce)

    p.stepSimulation()
    
    return body2feet_

def robot_quit():
    p.disconnect()


##This part of code is just to save the raw telemetry data.
fieldnames = ["t","FR","FL","BR","BL"]
with open('telemetry/data.csv','w') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames = fieldnames)
    csv_writer.writeheader()

def update_data():
    #take meassurement from simulation
    t , X = meassure.states()
    U , Ui ,torque = meassure.controls()
    
    with open('telemetry/data.csv','a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
        info = {"t" : t[-1],
                "FR" : Ui[2,0],
                "FL" : Ui[5,0],
                "BR" : Ui[8,0],
                "BL" : Ui[11,0]}
        csv_writer.writerow(info)
        
        
        
        
        
        
        
if __name__ == '__main__':
    dT = 0.005
    bodyId, jointIds = robot_init( dt = dT, body_pos = [0,0,0.18], fixed = False )
    pybulletDebug = pybulletDebug()
    robotKinematics = robotKinematics()
    trot = trotGait()
    meassure = systemStateEstimator(bodyId)

    """initial foot position"""
    #foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
    Xdist, Ydist, height = 0.18, 0.15, 0.10
    #body frame to foot frame vector
    bodytoFeet0 = np.matrix([[ Xdist/2. , -Ydist/2. , height],
                            [ Xdist/2. ,  Ydist/2. , height],
                            [-Xdist/2. , -Ydist/2. , height],
                            [-Xdist/2. ,  Ydist/2. , height]])

    offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
    footFR_index, footFL_index, footBR_index, footBL_index = 3, 7, 11, 15
    T = 0.5 #period of time (in seconds) of every step
 
    N_steps=50000
    N_par = 8
    out = np.zeros((N_par+1,N_steps))

    # start record video
    #p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot.mp4")
    for k_ in range(0,N_steps):
        lastTime = time.time()
        pos , orn , L , angle , Lrot , T , sda= pybulletDebug.cam_and_robotstates(bodyId)
        #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
        bodytoFeet = trot.loop( L , angle , Lrot , T , offset , bodytoFeet0 , sda)
        # out[0,k_]=k_*dT
        # out[1:4,k_]=bodytoFeet[0]
        # out[4:7,k_]=bodytoFeet[1]
        # out[7,k_]=L
        # out[8,k_]=Lrot
        # #import IPython;IPython.embed()
        robot_stepsim( bodyId, pos, orn, bodytoFeet )
        update_data()
        print(time.time() - lastTime)
    robot_quit()



