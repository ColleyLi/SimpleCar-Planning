import numpy as np
import os
import matplotlib.pyplot as plt

file_name = '../exec/MPC_path.txt'

MPC_path = list()
with open(file_name) as path_file:
    for line in path_file:
        MPC_path.append(line.strip('[ ]\t\n').split('\t'))
data = np.array(MPC_path, dtype=float).reshape(-1, 4)
#print(data[:10, 1:3])
fig, axe = plt.subplots()
line, = axe.plot(data[:, 1], data[:, 2], 'green', linewidth=2, label='path')

plt.show()
