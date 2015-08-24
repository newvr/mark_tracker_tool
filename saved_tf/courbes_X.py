#!/usr/bin/python

import numpy
import csv
from matplotlib import pyplot as plt
import math
import cv2

string1 = "coucou.txt"
fichier1 = numpy.genfromtxt(string1, skiprows=0, delimiter=' ')

temps = fichier1[:, 0]
x_speed = fichier1[:, 1]
y_speed = fichier1[:, 2]
z_speed = fichier1[:, 3]
tot = fichier1[:, 4]


xmax = max([x_speed.max(), y_speed.max(), z_speed.max(), tot.max()]) + 0.1

init = temps[0]
maxi = temps.max() - init

plt.plot(temps - init, x_speed, 'b', label='x_speed')
plt.plot(temps - init, y_speed, 'g', label='y_speed')
plt.plot(temps - init, tot, 'r', label='tot')

plt.axis([0, maxi, -xmax - 0.1, xmax + 0.1])

plt.xlabel("temps")
plt.ylabel("vitesse")
plt.grid(True)
plt.legend()

plt.show()
