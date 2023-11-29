#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import sin, cos, exp, sqrt, pi
from numpy import array, dot

from Robot3WD.Robot3WD import Robot


class myRobot(Robot):

    def __init__(
        self,
        sampling_period,
        wheel_radius=None,
        L=None,
        ):
        Robot.__init__(self, sampling_period, wheel_radius, L)

    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        matrix = array([[sin(theta), -cos(theta), -L], [cos(pi / 6
                       + theta), sin(pi / 6 + theta), -L], [-cos(pi / 6
                       - theta), sin(pi / 6 - theta), -L]])
        dot_product = matrix.dot(p_dot)
        wheel_angular_velocities = 1 / wheel_radius * dot_product
        return wheel_angular_velocities

    def move_left(self, vx, theta):
        p_dot = array([-vx, 0.0, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)

    def move_forward(self, vy, theta):
        p_dot = array([0.0, vy, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)

    def move_backward(self, vy, theta):
        p_dot = array([0.0, -vy, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)

    def move_right(self, vx, theta):
        p_dot = array([vx, 0.0, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)

    def rotate_CCW(self, w, theta):
        p_dot = array([0.0, 0.0, w]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)

    def rotate_CW(self, w, theta):
        p_dot = array([0.0, 0.0, -w]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)

    def forward_kinematics(self, wheel_angular_velocities, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        M = array([[2 * sin(theta), 2 * cos(theta + pi / 6), -2
                  * sin(theta + pi / 3)], [-2 * cos(theta), 2
                  * cos(theta - pi / 3), 2 * cos(theta + pi / 3)], [-1
                  / L, -1 / L, -1 / L]])
        p_dot = wheel_radius / 3 * dot(M, wheel_angular_velocities)
        return p_dot

    def motor_sat(self, wheel_angular_velocities, limit_value):
        wheel_angular_velocities_bar = array([0.0, 0.0, 0.0])

        if wheel_angular_velocities[0] < -limit_value:
            wheel_angular_velocities_bar[0] = -limit_value
        elif wheel_angular_velocities[0] > limit_value:
            wheel_angular_velocities_bar[0] = limit_value
        else:
            wheel_angular_velocities_bar[0] = \
                wheel_angular_velocities[0]
        if wheel_angular_velocities[1] < -limit_value:
            wheel_angular_velocities_bar[1] = -limit_value
        elif wheel_angular_velocities[1] > limit_value:
            wheel_angular_velocities_bar[1] = limit_value
        else:
            wheel_angular_velocities_bar[1] = \
                wheel_angular_velocities[1]
        if wheel_angular_velocities[2] < -limit_value:
            wheel_angular_velocities_bar[2] = -limit_value
        elif wheel_angular_velocities[2] > limit_value:
            wheel_angular_velocities_bar[2] = limit_value
        else:
            wheel_angular_velocities_bar[2] = \
                wheel_angular_velocities[2]
        return wheel_angular_velocities_bar


#### end of myrobot Class ###

# Define the HMatrix function

def HMatrix(q):

    H = array([[cos(q[2]), -sin(q[2]), q[0]], [sin(q[2]), cos(q[2]),
              q[1]], [0, 0, 1]])
    return H


# Define the Vraw_to_distance function

def Vraw_to_distance(Vraw):
    d = 0.62 * exp(-0.0539 * sqrt(Vraw))
    return d


#### end of myrobot.py ###
