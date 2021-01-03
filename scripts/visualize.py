# from mpl_toolkits.mpl import Axes3D
import numpy
import matplotlib.pyplot as plt

"""
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(data[:,1],data[:,2],data[:,3],'.-')
# plt.show()
"""

# Load data from different approaches and scenarios

data1 = numpy.loadtxt('../bin/simple_car_path.txt')
data2 = numpy.loadtxt('../bin/simple_car_path_app.txt')
data3 = numpy.loadtxt('../bin/simple_car_path_geometric.txt')
data4 = numpy.loadtxt('../bin/simple_car_path_geometric_app.txt')

# Plot path(note: grouped as 1 with 3, 2 with 4)
fig, axes = plt.subplots(2)
line1, = axes[0].plot(data1[:,0], data1[:,1], 'green', linewidth=2, label='path')
line2, = axes[1].plot(data2[:,0], data2[:,1], 'green', linewidth=2, label='path')
line3, = axes[0].plot(data3[:,0], data3[:,1], 'blue', linewidth=2, label='path')
line4, = axes[1].plot(data4[:,0], data4[:,1], 'blue', linewidth=2, label='path')

plt.show()
