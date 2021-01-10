# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 12:38:18 2021

@author: Yulei Qiu
"""
import numpy as np
import matplotlib.patches as patches

def plot_vehicle_state(ax, state):
    # rotation, translation
    ct = np.cos(state[2])
    st = np.sin(state[2])
    R = np.array([[ct, -st], [st, ct]])
    T = np.array([state[0], state[1]])
    
    slong = 10 # longitudinal size (decimeter)
    slat = 10 # lateral size (decimeter)
    
    # car outset
    lines = box_in_frame(ax, 0, 0, slat, slong, R, T, c='k')
    return lines

def box_in_frame(ax, cx, cy, w, h, R, T, c='k'):
    # car outset
    points = np.array([
        [1, -1, -1,  1,  1],
        [-1, -1,  1,  1, -1]
    ])
    points[0,:] = points[0,:] * w/2. + cx;
    points[1,:] = points[1,:] * h/2. + cy;
    lines = plot_in_frame(ax, points, R, T, c=c)
    return lines

def plot_in_frame(ax, points, R, T, c):
    # apply transformation
    points = R.dot(points)
    lines, = ax.plot(points[0,:] + T[0], points[1,:] + T[1], c=c)
    return lines

def plot_arrow(ax, state, state_next):
    L = 15
    dx = L*np.cos(state[2])
    dy = L*np.sin(state[2])
    arrow = patches.Arrow( state[0], state[1], dx, dy, width=5 )
    a = ax.add_patch(arrow)
    return a