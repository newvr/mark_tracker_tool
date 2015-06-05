#!/usr/bin/python

import numpy
import csv
from matplotlib import pyplot as plt
import math as m
import cv2

string1 = "test_vitesse_old.txt"
fichier1 = numpy.genfromtxt(string1, skiprows=0, delimiter=' ')

temps = fichier1[:, 0]
x_speed = fichier1[:, 1] * 10 ** 9
y_speed = fichier1[:, 2] * 10 ** 9
z_speed = fichier1[:, 3] * 10 ** 9

init = temps[0]
maxi = temps.max() - init

plt.plot(temps - init, x_speed, 'b', label='x_speed')
plt.plot(temps - init, y_speed, 'g', label='y_speed')
#plt.plot(temps - init, z_speed, 'r', label='z_speed')

plt.axis([0, maxi, -1, 1])


plt.xlabel("temps")
plt.ylabel("vitesse")
plt.grid(True)
plt.legend()

plt.show()
