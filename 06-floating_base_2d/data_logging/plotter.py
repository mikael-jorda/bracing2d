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
commanded_torques = file[:,3:8]
f_sensed = file[:,8:10]
# cmap = ['r','k']
# 

norm1_torques = np.sum(np.abs(commanded_torques[:,1::]), 1)
norm2_torques = np.sqrt(np.sum(commanded_torques[:,1::]*commanded_torques[:,1::], 1))

norm2_xerror = np.sqrt(np.sum(x_error*x_error, 1))

reference_index = 1000;
power_ratio = norm1_torques/norm1_torques[reference_index]
power_ratio_norm2 = norm2_torques/norm2_torques[reference_index]

plt.figure(1)
plt.plot(commanded_torques)
plt.title("Command Torques")

plt.figure(2)
plt.plot(norm1_torques)
plt.title("Total motor power")

plt.figure(4)
plt.plot(x_error[:,0])
plt.title("y error")

plt.figure(5)
plt.plot(f_sensed)
plt.title("f sensed")

plt.figure(6)
plt.plot(power_ratio)
plt.title("power ratio")

plt.figure(7)
plt.plot(power_ratio_norm2)
plt.title("power ratio norm 2")

plt.show()


