#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 16:27:16 2020

@author: linux-asd
"""
import numpy as np


def convert(FR_angles, FL_angles, BR_angles, BL_angles):
# It might change for other servomotors.
    pulse = np.empty([12])
    #FR
    pulse[0] = int(-10.822 * np.rad2deg(-FR_angles[0])) + 925
    pulse[1] = int(-10.822 * np.rad2deg(FR_angles[1])) + 2350
    pulse[2] = int(10.822 * (np.rad2deg(FR_angles[2]) + 90)) + 1000
    #FL
    pulse[3] = int(10.822 * np.rad2deg(FL_angles[0])) + 985
    pulse[4] = int(10.822 * np.rad2deg(FL_angles[1])) + 600
    pulse[5] = int(-10.822 * (np.rad2deg(FL_angles[2]) + 90)) + 1150
    #BR
    pulse[6] = int(10.822 * np.rad2deg(-BR_angles[0])) + 1025
    pulse[7] = int(-10.822 * np.rad2deg(BR_angles[1])) + 2325
    pulse[8] = int(10.822 * (np.rad2deg(BR_angles[2]) + 90)) + 1100
    #BL
    pulse[9] = int(-10.822 * np.rad2deg(BL_angles[0])) + 900
    pulse[10] = int(10.822 * np.rad2deg(BL_angles[1])) + 710
    pulse[11] = int(-10.822 * (np.rad2deg(BL_angles[2]) + 90)) + 1050
    return pulse
