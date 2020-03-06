#!/usr/bin/env python

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys

# data file to read given as argument
if len(sys.argv) < 2:
	print "Give the name of the file to read as an argument\n"
	exit()

file = np.loadtxt(sys.argv[1] ,skiprows=1)

time = file[:,0]
x_error = file[:,1:3]
commanded_torques = file[:,3:7]
# cmap = ['r','k']
# 

norm2_torques = np.sqrt(np.sum(commanded_torques*commanded_torques, 1))

norm2_xerror = np.sqrt(np.sum(x_error*x_error, 1))

plt.figure(1)
plt.plot(commanded_torques)
plt.title("Command Torques")

plt.figure(2)
plt.plot(norm2_torques)
plt.title("Total motor power")

plt.figure(3)
plt.plot(norm2_xerror)
plt.title("norm x error")

plt.figure(4)
plt.plot(x_error[:,0],label='y')
plt.plot(x_error[:,1],label='z')
plt.title("x error")
plt.legend()

plt.show()


