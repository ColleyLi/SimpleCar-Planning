# -*- coding: utf-8 -*-
"""
Visualize the path using Matplotlib. Show the animation of tracking the path.

@author: Yulei Qiu
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import plot_square as ps

# Load data from different approaches and scenarios
data1 = np.loadtxt('../data/simple_car_path_geometric.txt')
data2 = np.loadtxt('../data/simple_car_path_geometric_app.txt')
data3 = np.loadtxt('../data/output_states.txt')
data4 = np.loadtxt('../data/output_states_app.txt')

# Plot path(note: grouped as 1 with 3, 2 with 4)
# =============================================================================
# fig, axes = plt.subplots(1, 2, figsize=(10, 4))
# line1, = axes[0].plot(data1[:,0], data1[:,1], 'orange', linewidth=1.5, alpha=0.8, label='ref')
# line2, = axes[1].plot(data2[:,0], data2[:,1], 'orange', linewidth=1.5, alpha=0.8, label='ref')
# line3, = axes[0].plot(data3[:,0], data3[:,1], 'blue', linewidth=1.5, alpha=0.8, label='mpc')
# line4, = axes[1].plot(data4[:,0], data4[:,1], 'blue', linewidth=1.5, alpha=0.8, label='mpc')
# axes[0].legend(handles=[line1, line3])
# axes[1].legend(handles=[line2, line4])
# =============================================================================
fig = plt.figure(figsize=[10,10])
ax = fig.gca()
ax.plot(data1[:,0], data1[:,1], 'orange', linewidth=1.5, alpha=0.8, label='ref')
ax.set_aspect("equal")

ox, oy = [], []
# obstacles
# transform the index from 2D vector to the grid: 20 - i
for i in np.arange(90, 100+1, 1):
	for j in np.arange(90, 100+1, 1):
		oy.append(i)
		ox.append(j)
for i in np.arange(0, 120+1, 1):
	for j in np.arange(50, 55+1, 1):
		oy.append(i)
		ox.append(j)
for i in np.arange(100, 200+1, 1):
	for j in np.arange(150, 155+1, 1):
		oy.append(i)
		ox.append(j)
# space border		
for i in np.arange(0, 200+1, 1):
	for j in [0, 200]:
		oy.append(i)
		ox.append(j)
for i in [0, 200]:
	for j in np.arange(0, 200+1, 1):
		oy.append(i)
		ox.append(j)
ax.plot(ox, oy, 'sk')

# start and end point
ax.scatter(20.0, 20.0, c='cyan')
ax.scatter(180.0, 180.0, c='magenta')

# arrow = patches.Arrow( data3[0,0], data3[0,1], 10*(data3[0+1,0]-data3[0,0]), 10*(data3[0+1,1]-data3[0,1]) )
# a = ax.add_patch(arrow)

plt.pause(5.0)
for i in range(0, data3.shape[0], 80):
    line_square = ps.plot_vehicle_state(ax, data3[i,:])
    a = ps.plot_arrow(ax, data3[i,:], data3[i+20,:])
    plt.pause(.01)
    line_square.remove()
    a.remove()

plt.show()