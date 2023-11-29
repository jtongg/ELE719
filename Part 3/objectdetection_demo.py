from time import time
from math import sqrt, pi, sin, cos, atan2
from numpy import array, dot, zeros, sum
from numpy.linalg import norm

from myRobot import *

dT = 0
# Set up sampling period T_s and stopping time T_f
T_s = 0.02
T_f = 30.0

# Set up initial pose
x_0 = 0.0
y_0 = 0.0
theta_0 = pi/6

# Set up goal position
x_f = 0.7
y_f = 1.0
theta_f = pi/2 #assumed

# Set up p_0 and p_f
p_0 = array([x_0,y_0,theta_0]).T
p_f = array([x_f,y_f,theta_f]).T

# Set up error tolerance
#epsilon = sqrt( 2.0*(0.5/100.0)**2)
epsilon = 0.005

# set up d_free and d_min
d_min = 0.08
d_free = 1.25*d_min

# Set up controller gains
k_rho = 0.5
k_beta = -0.5
k_alpha = (2.0/pi)*k_rho - (5.0/3.0)*k_beta + 0.5

# Initialize vector pr_dot to be used for inverse kinematics
pr_dot = array([0.0,0.0,0.0]).T

# Initialize vector d for storing the sensor measurements d_i 
d = array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]).T

# Initialize a 6x3 matrix u_iR.  The vector u_i^R is assigned the ith column
# of this matrix.  See Eq. (4.6) in lab manual.
u_iR = zeros(shape=(3,6))

# Sensor frame location as per Table 4.1
R = 0.13
halfR = R/2.0
root3Rover2 = sqrt(3.0)*R/2.0

# Sensor frame location as per Table 4.1
sensor_loc = array([
   [-halfR      , -root3Rover2, -(2.0/3.0)*pi],
   [-root3Rover2, halfR       , (5.0/6.0)*pi],
   [0.0           , R           , (1/2)*pi],
   [halfR       , root3Rover2 , (1.0/3.0)*pi],
   [root3Rover2 , halfR       , (1.0/6.0)*pi],
   [root3Rover2 , -halfR      , -(1.0/6.0)*pi]
])

# open a file to store robot pose for plotting later
f = open('pose.csv', 'w')

# Initial robot and other variables
robot = myRobot(T_s)

robot.initialize(theta_0)

p = p_0
dp = p_f - p_0
rho = norm(dp[0:2])
goal_reached = False
elapsed_time = 0.0

# Set start time
start_time = time()

# Control loop
while ( (not goal_reached) and (elapsed_time < T_f) ):

    robot.get_readings_update()

    # save data to file for plotting
    f.write('%10.3e %10.3e % 10.3e %10.3e %10.3e\n' % (elapsed_time, p[0], p[1], p[2], rho))

    theta = p[2]

    # Use forward kinematics to get p_dot
    p_dot = robot.forward_kinematics(robot.angular_velocities, theta)

    # Determine angle alpha (remember to use atan2 instead of atan)
    alpha = atan2((y_f-p[1]),(x_f-p[0]))-theta
    
    u_R = array([0.0,0.0,0.0])

    # Get sensor measurements w.r.t. sensor frame
    for i in range(0,6):
       d[i] = Vraw_to_distance(robot.ir_sensors_raw_values[i])
       dT = dT + d[i]

    # Check if obstacle is present
    # If true, determine temporary goal position and use it to calculate
    # rho bar and alpha bar.  See Eqs. 4.6 to 4.11.
    if min(d) <= d_min: 
        
        for i in range(0,6):
        
            if d[i] > d_free:
        
                HSR = HMatrix(sensor_loc[i,])
                delta_iS = array([d[i],0,1]).T
                delta_iR = dot(HSR,delta_iS)
                mag = sqrt(delta_iR[0]**2 + delta_iR[1]**2 + delta_iR[2]**2)
                u_iR[:,i] = delta_iR/mag
        
        for i in range (0,6):
            u_R = u_R + (d[i]/dT)*(u_iR[:,i])
        
        #q = array([y_f-y_0,x_f-x_0,theta_0-theta_f]).T
        HR0 = HMatrix(p)
        u_0 = dot((HR0),(u_R))
        
        
        pT = d_free*u_0
        
        rho = sqrt((pT[0]-p[0])**2 + (pT[1]-p[1])**2)
        
        alpha = atan2((pT[1]-p[1]),(pT[0]-p[0]))-theta
    
    beta = -(theta + alpha)

    # Determine kinear and angular velocities of the robot body (v, w)
    # using the control law given in Eqs. (4.12) and (4.13)
    
    v = k_rho*rho
    w = k_alpha*alpha + k_beta*beta

    # Determine pr_dot
    pr_dot[0] = v*cos(theta)
    pr_dot[1] = v*sin(theta)
    pr_dot[2] = w

    # Now use Inverse Kinematics to determine wheel ref velocities
    wheel_ref_vel = robot.inverse_kinematics(pr_dot, theta) 
    
    # Apply motor limits
    wheel_ref_vel = robot.motor_sat(wheel_ref_vel, 5.0*pi)

    # Execute motion
    robot.set_angular_velocities(wheel_ref_vel)

    # Odometry update
    p = p+p_dot*T_s

    # Replace calculated update for theta with measured value from IMU
    p[2]= robot.orientation

    # Check to see if goal is reached
    dp = p_f - p
    rho = norm(dp[0:2])
    goal_reached = ( rho <= epsilon)

    # time update
    elapsed_time = time() - start_time


print('ELPASED TIME = %s rho = %s' % (elapsed_time, rho))

# Either goal is reached or current_time > T_f
if goal_reached:
   print('Goal is reached, error norm is', rho)
else:
   print('Failed to reach goal, error norm is', rho)

robot.stop()
robot.close()

f.close()
