#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 20:31:07 2020

@author: miguel-asd
"""

import pandas as pd
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('dark_background')

index = count()


def animate(i):
    data = pd.read_csv('telemetry/data.csv')
    x = data['t']
    y1 = data['roll']
    y2 = data['pitch']
    xlim = x.values[-1]
    
    plt.clf()
    plt.subplot(121)
    plt.plot(x, y1, label='roll')
    plt.xlim(xlim-30,xlim)
    plt.title('Body orientation.')
    plt.legend(loc='upper left')
    
    plt.subplot(122)
    plt.plot(x, y2, label='pitch')
    plt.xlim(xlim-30,xlim)
    plt.title('Body orientation.')
    plt.legend(loc='upper left')
    
    plt.tight_layout()
    

ani1 = FuncAnimation(plt.gcf(), animate, interval=25)

plt.tight_layout()
plt.show()

