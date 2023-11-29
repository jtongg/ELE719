from numpy import array
from numpy.linalg import norm
from math import sqrt, pi, atan2
from time import time

from Robot3WD.PID import PIDControllers

from myRobot import *

# Set up sampling period and stopping time
T_s = 0.02
T_f = 20.0

# Set up initial pose
x_0 = 0.0
y_0 = 0.0
theta_0 = pi/6 

# Set up final pose
x_f = 0.7
y_f = 1.0
theta_f = pi/2

# set up p_0, p_f
p_0 = array([x_0, y_0, theta_0]).T
p_f = array([x_f, y_f, theta_f]).T

# Set up stopping criterion
epsilon = sqrt( 2.0*(0.5/100.0)**2 + (1.0*pi/180.0)**2 )
print('epsilon =', epsilon)

# Initialize PID controller
Kp = array([0.2, 0.2, 0.2]).T
Ki = array([0.2, 0.2, 0.2]).T
Kd = array([0.01, 0.01, 0.01]).T
pid = PIDControllers(Kp, Ki, Kd, T_s)

# Initialize robot and other variables

robot = myRobot(T_s)
robot.initialize(theta_0)

goal_reached = False

current_time = 0.0

p = p_0
error_norm = norm(p_f - p)

# open a file to store data for plotting
f = open('pose.csv', 'w')

# Set start time to current time
start_time = time()

# Control Loop
while ( (not goal_reached) and (current_time < T_f) ): 
   robot.get_readings_update()

   # write data to file for plotting
   f.write('%10.3e %10.3e %10.3e %10.3e %10.3e\n' % (current_time, p[0], p[1], p[2], error_norm) )
   # get current robot orientation
   theta = robot.orientation

   # Determine p_dot using forward kinematics
   p_dot = robot.forward_kinematics(robot.angular_velocities, theta)
   
   # Determine pd_dot (desired p_dot) using PID control law
   pd_dot = pid(p_f, p)

   # Determine desired wheel velocities using inverse kinematics
   w = robot.inverse_kinematics(pd_dot, theta)

   # Apply motor limits
   w = robot.motor_limit(w,5*pi)

   # Execute the motion
   robot.set_angular_velocities(w)

   # odometry update
   p = p + pd_dot*T_s
   # check to see if goal is reached
   error_norm = norm(p_f - p)
   goal_reached = (error_norm <= epsilon)

   # time update
   current_time = time() - start_time


print('Elapsed time = ', time()-start_time)

# Either goal is reached or current_time > T_f
if goal_reached:
   print('Goal is reached, error norm is', error_norm)
else:
   print('Failed to reach goal, error norm is', error_norm)

robot.stop()
robot.close()

# close file
f.close()
