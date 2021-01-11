# -*- coding: utf-8 -*-
"""
Visualize the path using Matplotlib. Show the animation of tracking the path.

@author: Jianfeng Cui, Yulei Qiu
"""

import numpy as np
import matplotlib.pyplot as plt
import plot_square as ps

# load data from different approaches and scenarios
data1 = np.loadtxt('../data/simple_car_path_geometric.txt')
data3 = np.loadtxt('../data/output_states.txt')

# plot the reference track
fig = plt.figure(figsize=[10,10])
ax = fig.gca()
ax.plot(data1[:,0], data1[:,1], 'orange', linewidth=1.5, alpha=0.8, label='ref')
ax.set_aspect("equal")

# obstacles
ox, oy = [], []
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

# animate the tracking process
plt.pause(1.0)
for i in range(0, data3.shape[0], 80):
    line_square = ps.plot_vehicle_state(ax, data3[i,:])
    a = ps.plot_arrow(ax, data3[i,:], data3[min(i+20, data3.shape[0]-1),:]) # if out of bounds
    plt.pause(.02)
    line_square.remove()
    a.remove()

plt.show()
