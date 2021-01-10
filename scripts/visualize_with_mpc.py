"""
Visualize the path using Matplotlib
@author: Jianfeng Cui
"""
import numpy as np
import matplotlib.pyplot as plt

# Load data from different approaches and scenarios
data1 = np.loadtxt('../data/simple_car_path_geometric.txt')
data2 = np.loadtxt('../data/simple_car_path_geometric_app.txt')
data3 = np.loadtxt('../data/output_states.txt')
data4 = np.loadtxt('../data/output_states_app.txt')

# Plot path(note: grouped as 1 with 3, 2 with 4)
fig, axes = plt.subplots(1, 2, figsize=(10, 4))
line1, = axes[0].plot(data1[:,0], data1[:,1], 'orange', linewidth=1.5, alpha=0.8, label='ref')
line2, = axes[1].plot(data2[:,0], data2[:,1], 'orange', linewidth=1.5, alpha=0.8, label='ref')
line3, = axes[0].plot(data3[:,0], data3[:,1], 'blue', linewidth=1.5, alpha=0.8, label='mpc')
line4, = axes[1].plot(data4[:,0], data4[:,1], 'blue', linewidth=1.5, alpha=0.8, label='mpc')
axes[0].legend([line1, line3])
axes[1].legend([line2, line4])

# map with obstacles

# for(int i = 0; i < GRID_ROWS; i++) { 
# 	for(int j = 0; j < GRID_COLS; j++) {
# 		if ((i >= 9 && i <= 11 && j >= 9 && j <= 11 ) || 
# 				(i >= 8 && i <= 19 && j >= 5 && j <= 6) ||
# 					(i >= 0 && i <= 10 && j >= 15 && j <= 16)){
# 			map_[i][j] = 0;
# 		}
# 		else {
# 			map_[i][j] = 1;
# 		}
# 	}
# }

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
axes[0].plot(ox, oy, 'sk')

axes[0].scatter(20.0, 20.0, c='cyan')
axes[0].scatter(180.0, 180.0, c='magenta')

plt.show()
