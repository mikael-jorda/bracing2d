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
x_current = file[:,1:3]
x_desired = file[:,3:5]
commanded_torques = file[:,5:10]
f_sensed = file[:,10:12]
# cmap = ['r','k']
# 

x_error = x_current - x_desired;

norm1_torques = np.sum(np.abs(commanded_torques[:,1::]), 1)
# norm1_torques = np.sum(np.abs(commanded_torques[:]), 1)
norm2_torques = np.sqrt(np.sum(commanded_torques[:,1::]*commanded_torques[:,1::], 1))

norm2_xerror = np.sqrt(np.sum(x_error*x_error, 1))

reference_index = 1000;
power_ratio = norm1_torques/norm1_torques[reference_index]
power_ratio_norm2 = norm2_torques/norm2_torques[reference_index]

without_bracing = np.arange(2000,7000)
with_bracing = np.arange(14500,19500)

plt.figure(0)
plt.subplot(3,2,1)
plt.plot(time[without_bracing], x_current[without_bracing,0])
plt.plot(time[without_bracing], x_desired[without_bracing,0], 'r--')
plt.ylim(3.0,3.5)
plt.subplot(3,2,2)
plt.plot(time[with_bracing], x_current[with_bracing,0])
plt.plot(time[with_bracing], x_desired[with_bracing,0], 'r--')
plt.ylim(3.0,3.5)
plt.subplot(3,2,3)
plt.plot(time[without_bracing], f_sensed[without_bracing,1])
plt.plot(time[without_bracing], -10.0 * np.ones(len(without_bracing)), 'r--')
plt.ylim(-15.0,-5.0)
plt.subplot(3,2,4)
plt.plot(time[with_bracing], f_sensed[with_bracing,1])
plt.plot(time[with_bracing], -10.0 * np.ones(len(with_bracing)), 'r--')
plt.ylim(-15.0,-5.0)
plt.subplot(3,2,5)
plt.plot(time[without_bracing], norm1_torques[without_bracing])
plt.ylim(0.0,70.0)
plt.subplot(3,2,6)
plt.plot(time[with_bracing], norm1_torques[with_bracing])
plt.ylim(0.0,70.0)

# plt.figure(1)
# plt.plot(commanded_torques)
# plt.title("Command Torques")

# plt.figure(2)
# plt.plot(norm1_torques)
# plt.title("Total motor power")

# plt.figure(4)
# plt.plot(x_error[:,0])
# plt.title("y error")

# plt.figure(5)
# plt.plot(f_sensed)
# plt.title("f sensed")

# plt.figure(6)
# plt.plot(power_ratio)
# plt.title("power ratio")

# plt.figure(7)
# plt.plot(power_ratio_norm2)
# plt.title("power ratio norm 2")

plt.show()


