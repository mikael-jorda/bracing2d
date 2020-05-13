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
gravity = file[:,7:11]
effective_gravity = file[:,11:15]
alpha = file[:,15]
colinearity = file[:,16]

norm1_torques = np.sum(np.abs(commanded_torques), 1)
norm2_torques = np.sqrt(np.sum(commanded_torques * commanded_torques, 1))
norm2_xerror = np.sqrt(np.sum(x_error*x_error, 1))

torque_ref_step = 500;

power_ratio = norm1_torques/norm1_torques[torque_ref_step]
power_ratio_2 = norm2_torques/norm2_torques[torque_ref_step]

# print(norm1_torques[-2]/norm1_torques[2])

plt.figure(1)
plt.plot(commanded_torques)
plt.title("Command Torques")

# plt.figure(2)
# plt.plot(norm1_torques)
# plt.title("Total motor power")

plt.figure(3)
plt.plot(norm2_xerror)
plt.title("norm x error")

plt.figure(33)
plt.plot(power_ratio)
plt.title("power ratio")

plt.figure(34)
plt.plot(power_ratio_2)
plt.title("power ratio 2")

plt.figure(35)
plt.plot(alpha)
plt.title("alpha")

plt.figure(40)
plt.plot(colinearity)
plt.title("colinearity")

# plt.figure(4)
# plt.plot(x_error[:,0],label='y')
# plt.plot(x_error[:,1],label='z')
# plt.title("x error")
# plt.legend()

# plt.figure(5)
# plt.plot(gravity)
# plt.title("gravity")

# plt.figure(6)
# plt.plot(effective_gravity)
# plt.title("effective gravity")

plt.show()


