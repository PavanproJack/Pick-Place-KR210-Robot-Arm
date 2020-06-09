[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# KR210 Robotic arm - Pick & Place project
This project is adopted from Udacity - Robotics NanoDegree Program. Modified and Implemented the Pick and Place functionality with new approaches.

This documentation demonstrates my perception and the steps involved in completing this project. 

Initial setup of ROS+Gazebo with VMware Fusion can be found in the Installation_instr.md file in this repo.

<img src = "/Kuka_KR210.jpg" width = "150">  

## Problem Statement:
Identify target object on the shelf, plan and perform a clean movement towards the object to grasp the object and place the in in the bin.

### Tools used:
RViz (or rviz) stands for ROS Visualization tool or ROS Visualizer. RViz is our one stop tool to visualize all three core aspects of a robot: Perception, Decision Making, and Actuation.

Gazebo: is a physics based high fidelity 3D simulator for robotics. Gazebo provides the ability to accurately simulate one or more robots in complex indoor and outdoor environments filled with static and dynamic objects, realistic lighting, and programmable interactions.

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

| Frame(n-1 -> n) | a | 𝜶 | d | θ |
|-------|--------|---------|--------|---------|
| 0 -> 1 | 0.000 | 0 | 0.75 | θ1 |
| 1 -> 2 | 0.350 | 90| 0.00 | θ2 | 
| 2 -> 3 | 1.250 | 0 | 0.00 | θ3+90 | 
| 3 -> 4 | -0.054| 90| 1.50 | θ4 | 
| 4 -> 5 | 0.0000|-90| 0.00 | θ5 |
| 5 -> 6 | 0.000 | 90| 0.00 | θ6 |
| 6 -> G | 0.000 |  0| 0.303|  0 |


## Inverse Kinematics of 6R KR210 arm:
<img src = "/misc_images/KukaKR210_schematic.jpg" width = "700">   

### Kinematic Decoupling:
Kinematic decoupling is used to consider position and orientation problems independently. Geometric approach is used for positioning problem and Euler angle parameterisation is used for orientation problem.

Majority of six DOF manipualators used in industry have three neighboring joint axes intersect at a single point which makes up a spherical wrist. KR210 has a spherical wrist with its last three joint axes intersecting at a point called Wrist Center Wc. The position of the wrist center is governed by the first three joints. Let P be the position of gripper. R0_g is the rotation matrix obtained from Homogeneous transformation from the base to gripper.
```
Wrist center Wc = [P_x, P_y, P_z] - d7 * R0_g * [0 0 1]' and d7 from DH table is 0.303
```
### Euler composition of Rotations:
According to Euler, any orientation(gripper's) w.r.t some fixed reference frame(base) can always be described by 3 elementary rotations in a given sequence. One such sequence is the z-y-x intrinsic rotations. 

```
R0_g = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr
```
Here 'yaw', 'pitch' and 'roll' angles and P_x, P_y, P_z are provided by Ros message "geometry_msgs". "geometry_msgs" provides messages for common geometric primitives such as points, vectors, and poses.





Solve for the joint angles given the wrist center Wc in Geometric approach. 
#### Inverse Position: A Geometric Approach:

<img src = "/misc_images/Inverse Position Schematic.png" width = "500">  

From the figure attached, Theta1 can be found to be 
```
θ1 = atan2(Wc_y, Wc_x)
θ2 = ....................
θ3 = 

```
 

#### Inverse Orientation: 
Since Gripper is in same orientation with joint 6, R0_g = R0_6 and R3_6 = R3_g

```
R0_3 = R0_1 * R0_2 * R0_3
With Theta1, Theta2 and Theta3, R0_3 can be evaluate numerically. Now
R3_6 = R3_4 * R4_5 * R5_6. This cannot be evaluated until we know Theta4, Theta5 and Theta6
Also
R0_6 = R0_3 * R3_6
R3_g = Inverse(R0_3) * R0_6  that implies
R3_6 = Transpose(R0_3) * R0_6  since rotation matrices have same inverse and transpose.
```
Finally, calculating θ1, θ2, θ3, θ4, θ5, θ6 completes the inverse kinematics problem.



## References:
```
http://www4.cs.umanitoba.ca/~jacky/Robotics/Papers/spong_kinematics.pdf
https://www.youtube.com/watch?v=rA9tm0gTln8
https://www.mroelectric.com/blog/kuka-robot-arm/
```



## Support or Contact:
Happy to support through mail: kavvuripavankumar@gmail.com






