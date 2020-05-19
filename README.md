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

### DENAVIT-HARTENBERG (DH)CONVENTION:
The advantage of DH representation is that, it is a minimal representation in the sense of requiring the smallest number of parameters to describe the robots serial kinematic structure.

Under this convention rotations only about z-axis and translations along x-axis are allowed.

Frames assignment rules:
```
1. Z-axis is the axis of rotation
2. 
```


| Frame(i) | θ | 𝜶 | r | d |
|-------|--------|---------|--------|---------|
| 1 | θ1 | 90 | 0 | L1 |
| 2 | θ2 | 0 | L2 | 0 |
| 3 | θ3 | 0 | L3 | 0 |
| 4 | θ4 + 90 | -90 | L4 | 0 |
| 5 | θ5 | 0 | 0 | L5 |


## Inverse Kinematics of 6R KR210 arm:

### Kinematic Decoupling:
Kinematic decoupling is used to consider position and orientation problems independently. Geometric approach is used for positioning problem and Euler angle parameterisation is used for orientation problem.



## References:
```
http://www4.cs.umanitoba.ca/~jacky/Robotics/Papers/spong_kinematics.pdf

```



## Support or Contact:
Happy to support through mail: kavvuripavankumar@gmail.com






