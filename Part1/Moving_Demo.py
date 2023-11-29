import numpy as np
import math
import time
from numpy import array, dot

from myRobot import myRobot

theta0 = 30.0*math.pi/180.0
w = 2
run_time = 2.5
Ts = 0.02

myRobot = myRobot(Ts)

myRobot.initialize(theta0)

print('Moving forward')

elapsed_time = 0.0
t0 = time.time()

while (elapsed_time < run_time):
   myRobot.get_readings_update()
   theta = myRobot.orientation
   myRobot.move_forward(0.2, theta)
   elapsed_time = time.time() - t0

myRobot.stop()

print('Moving right')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   myRobot.get_readings_update()
   theta = myRobot.orientation
   myRobot.move_right(0.2, theta)
   elapsed_time = time.time() - t0

myRobot.stop()

print('Moving backward')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   myRobot.get_readings_update()
   theta = myRobot.orientation
   myRobot.move_backward(0.2, theta)
   elapsed_time = time.time() - t0

myRobot.stop()

print('Moving left')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   myRobot.get_readings_update()
   theta = myRobot.orientation
   myRobot.move_left(0.2, theta)
   elapsed_time = time.time() - t0

myRobot.stop()

print('Rotating CCW')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   myRobot.get_readings_update()
   theta = myRobot.orientation
   myRobot.rotate_CCW(2, theta)
   elapsed_time = time.time() - t0

print('Rotating CW')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   myRobot.get_readings_update()
   theta = myRobot.orientation
   myRobot.rotate_CW(2, theta)
   elapsed_time = time.time() - t0

myRobot.stop()

myRobot.close()
