#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 16:27:16 2020

@author: miguel-asd
"""
import numpy as np

def convert(FR_angles, FL_angles, BR_angles, BL_angles):
    pulse = np.empty([12])
    #FR
    pulse[0] = int(-10.822 * np.rad2deg(-FR_angles[0])) + 950
    pulse[1] = int(-10.822 * np.rad2deg(FR_angles[1])) + 2280
    pulse[2] = int(10.822 * (np.rad2deg(FR_angles[2]) + 90)) + 1000
    #FL
    pulse[3] = int(10.822 * np.rad2deg(FL_angles[0])) + 1020 
    pulse[4] = int(10.822 * np.rad2deg(FL_angles[1])) + 570
    pulse[5] = int(-10.822 * (np.rad2deg(FL_angles[2]) + 90)) + 1150
    #BR
    pulse[6] = int(10.822 * np.rad2deg(-BR_angles[0])) + 1060 
    pulse[7] = int(-10.822 * np.rad2deg(BR_angles[1])) + 2335 
    pulse[8] = int(10.822 * (np.rad2deg(BR_angles[2]) + 90)) + 1200
    #BL
    pulse[9] = int(-10.822 * np.rad2deg(BL_angles[0])) + 890
    pulse[10] = int(10.822 * np.rad2deg(BL_angles[1])) + 710
    pulse[11] = int(-10.822 * (np.rad2deg(BL_angles[2]) + 90)) + 1050
    return pulse