# ELE 719: Introduction to Robotics                                                                                                        Sept-Dec 2022

The goal of the project was to model and control a 3-wheeled mobile robot as detailed below using Python programming on a Linux platform to control movement, speed and orientation. 

The movement of the robot would be based on DC motor control and calculated utilizing the inverse and forward kinematics equations to control and limit the motor speeds. 

![image](https://github.com/jtongg/ELE719/assets/118993878/d3890ea3-b85d-41cb-868a-30783110d47d)

The project was split up into 3 different parts. 

 # Part 1: Moving Demo 
 - This part involved the basic movements of the robot. The robot was to move forward, backwards, left, right and finally rotate CCW & CW.

# Part 2: PID Control of the Robot 
- Utilizing the basic movement functions defined in the previous part, the robot utilizes forward kinematics to determine the speeds required to move to a position defined on the ground. (Part 1)
- The robot will then utilize PID control to find and optimize the fastest route to travel to that set position by limiting the error (Part 2)

  # Part 3: Object Detection of the Robot
  - Utilizing IR sensors and sensor measurements the robot is to sense an object within a set distance, and if an object is determined to be there, move out of the way to reach desired point. 
