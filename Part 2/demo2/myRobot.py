from math import sin, cos, exp, sqrt, pi
from numpy import array, dot

from Robot3WD.Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period, wheel_radius=None, L=None):
        Robot.__init__(self, sampling_period, wheel_radius, L)

    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        phi = [[sin(theta),-cos(theta),-L],[cos(pi/6+theta),sin(pi/6+theta),-L],[-cos(pi/6-theta),sin(pi/6-theta),-L]]
        phi = dot(1/wheel_radius,phi)
        wheel_angular_velocities = dot(phi,p_dot)
        return wheel_angular_velocities

    def move_left(self, v, theta):

        p_dot = [-v, 0.0, 0.0]

        w = self.inverse_kinematics(p_dot, theta)

        self.set_angular_velocities(w)

    def move_forward(self, v, theta):
 
        p_dot = [0.0, v, 0.0]

        w = self.inverse_kinematics(p_dot, theta)

        self.set_angular_velocities(w)

    def move_backward(self, v, theta):

        p_dot = [0.0, -v, 0.0]

        w = self.inverse_kinematics(p_dot, theta)

        self.set_angular_velocities(w)

    def move_right(self, v, theta):

        p_dot = [v, 0.0, 0.0]

        w = self.inverse_kinematics(p_dot, theta)

        self.set_angular_velocities(w)

    def rotate_CCW(self, w, theta):

        p_dot = [0.0, 0.0, w]

        w = self.inverse_kinematics(p_dot, theta)

        self.set_angular_velocities(w)

    def rotate_CW(self, w, theta):
        p_dot = [0.0, 0.0, -w]

        w = self.inverse_kinematics(p_dot, theta)

        self.set_angular_velocities(w)

    def forward_kinematics(self, wheel_angular_velocities, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        phi = [[2*sin(theta),2*cos(theta+pi/6),-2*sin(theta+pi/3)],[-2*cos(theta),2*cos(theta-pi/3),2*cos(theta+pi/3)],[-1/L,-1/L,-1/L]]
        phi = dot(wheel_radius/3,phi)
        p_dot = dot(phi,wheel_angular_velocities)
        return p_dot

    def motor_limit(self, wheel_angular_velocities, limit_value):
        wheel_angular_velocities_bar = [0,0,0]
        for x in range(3):
            if (wheel_angular_velocities[x] > limit_value):
                wheel_angular_velocities_bar[x] = limit_value
            elif (wheel_angular_velocities[x] < -limit_value):
                wheel_angular_velocities_bar[x] = -limit_value
            else:
                wheel_angular_velocities_bar[x] = wheel_angular_velocities[x]	
        return wheel_angular_velocities_bar
