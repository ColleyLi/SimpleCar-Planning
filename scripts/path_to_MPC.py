# -*- coding: utf-8 -*-
"""
Created on Thu Dec 31 21:40:33 2020
@author: Yulei Qiu
"""

import numpy as np

a = np.loadtxt('../exec/simple_car_path_app.txt') # load the reference as a 2D ndarray

b = np.delete(a, np.s_[3:5], 1) # delete control elements
# b = a

# c = b[:,[5,0,1,2,3,4]]
c = b[:,[3,0,1,2]] # rearrange the columns

c[:,0] = np.cumsum(c[:,0], axis=0) # calculate the time stamp

d = np.savetxt('path.txt', c,  '%.5f') # save it to a .txt file
