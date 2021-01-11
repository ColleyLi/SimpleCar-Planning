# -*- coding: utf-8 -*-
"""
Visualize the path using Matplotlib. Show the animation of tracking the path.

@author: Jianfeng Cui, Yulei Qiu
"""

import numpy as np
import matplotlib.pyplot as plt
import plot_square as ps

# load data from different approaches and scenarios
data2 = np.loadtxt('../data/simple_car_path_geometric_app.txt')
data4 = np.loadtxt('../data/output_states_app.txt')

# plot the reference track
fig = plt.figure(figsize=[10,10])
ax = fig.gca()
ax.plot(data2[:,0], data2[:,1], 'orange', linewidth=1.5, alpha=0.8, label='ref')
ax.set_aspect("equal")

# start and end point
#ax.scatter(80.0, -400.0 + 440, c='cyan')
#ax.scatter(500.0, -80.0 + 440, c='magenta')

# animate the tracking process
plt.pause(1.0)
for i in range(0, data4.shape[0], 80):
    line_square = ps.plot_vehicle_state(ax, data4[i,:])
    a = ps.plot_arrow(ax, data4[i,:], data4[i+20,:])
    plt.pause(.02)
    line_square.remove()
    a.remove()

plt.show()
