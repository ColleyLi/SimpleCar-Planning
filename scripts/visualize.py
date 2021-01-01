# from mpl_toolkits.mpl import Axes3D
import numpy
import matplotlib.pyplot as plt

"""
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(data[:,1],data[:,2],data[:,3],'.-')
# plt.show()
"""

# data from manual scenario
#data = numpy.loadtxt('../exec/simple_car_path.txt')
# data from app scenario
data = numpy.loadtxt('../exec/simple_car_path_app.txt')

fig, axe = plt.subplots()
line, = axe.plot(data[:,0], data[:,1], 'green', linewidth=2, label='path')

plt.show()
