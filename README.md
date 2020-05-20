[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# KR210 Robotic arm - Pick & Place project
This project is adopted from Udacity - Robotics NanoDegree Program. Modified and Implemented the Pick and Place Functionality with new approaches.

This documentation demonstrates my perception and the steps involved in completing this project. 

Initial setup of ROS+Gazebo with VMware Fusion can be found in the Installation_instr.md file in this repo.

## Problem Statement:
Identify target object on the shelf, plan and perform a clean movement towards the object to grasp the object and place the in in the bin.

### Tools used:
RVIZ:

Gazebo:

MoveIT! is the software for building mobile manipulation applications.It integrates perception, kinematics, motion planning, trajectory processing and execution.

## Forward Kinematics of 6R KR210 arm:

### Denavit-Hartenberg (DH)Convention:
The advantage of DH representation is that, it is a minimal representation in the sense of requiring the smallest number of parameters to describe the robots serial kinematic structure.

Under this convention rotations only about z-axis and translations along x-axis are allowed.

Frames assignment rules:
Assign the frames assuming all the joint co-ordinates are zero. For eg. zero rotation for revolute and zero displacement for prismatic.
```
```
1. Label all the joints from 1 to n
2. Label all the links from 0 to n with 0 being the base link.
3. Z-axis is always the joint axis for both revoulte and prismatic joints. 
   Attach frames 0 to base link and frame 'g' to the gripper(end-effector).
4. Define common normals between the joint axes(Z_n-1 and Z_n) orthogonal to both the axes.
5. New X-axis(X_n) points along the co-normal and has its origin at the intersection of co-normal and new Z-axis.
6. 'd' is the depth along the previous joint's Z-axis(Z_n-1) to the common normal. 
   More simplified, it is the distance along    Z_n−1 from O_n−1(previous joint's origin) to the intersection of the X_n and      Z_n−1 axes (which is the new origin).
7. Theta is the angle about previous Z-axis(Z_n-1) to align its X-axis(X_n-1) with the new X-axis(X_n)
8. 'a' is length of the co-normal itself. Infact, it is the distance along X_n from O_n to the intersection of the X_n and Z_n−1 axes
9. Finally, Alpha is the rotation about new X_n axis to align Z_n-1 and Z_n axes.


### KR210 schematic diagram with frames attached:


### Digesting URDF file to populate DH table:
Unified Robot Description Format or urdf, is an XML format used in ROS for representing a robot model. We can use a urdf file to define a robot model, its kinodynamic properties, visual elements and even model sensors for the robot.

| Frame(i) | a | 𝜶 | d | θ |
|-------|--------|---------|--------|---------|
| 0 -> 1 | 0.000 | 0 | 0.75 | θ1 |
| 1 -> 2 | 0.350 | 90| 0.00 | θ2 | 
| 2 -> 3 | 1.250 | 0 | 0.00 | θ3+90 | 
| 3 -> 4 | -0.054| 90| 1.50 | θ4 | 
| 4 -> 5 | 0.0000|-90| 0.00 | θ5 |
| 5 -> 6 | 0.000 | 90| 0.00 | θ6 |
| 6 -> G | 0.000 |  0| 0.303|  0 |


## Inverse Kinematics of 6R KR210 arm:

### Kinematic Decoupling:
Kinematic decoupling is used to consider position and orientation problems independently. Geometric approach is used for positioning problem and Euler angle parameterisation is used for orientation problem.



## References:
```
http://www4.cs.umanitoba.ca/~jacky/Robotics/Papers/spong_kinematics.pdf
https://www.youtube.com/watch?v=rA9tm0gTln8

```



## Support or Contact:
Happy to support through mail: kavvuripavankumar@gmail.com






