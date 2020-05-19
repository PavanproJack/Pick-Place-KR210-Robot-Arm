#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from sympy import cos, sin, pi, simplify, symbols, sqrt, atan2
from sympy.matrices import Matrix
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  #Theta
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  #d-offset

        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  #
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # alpha twist angle..

        dh = {
            a0: 0,      alpha0: 0,      q1: q1,        d1: 0.75,
            a1: 0.35,   alpha1: pi/2,   q2: q2,        d2: 0.00,
            a2: 1.25,   alpha2: 0,      q3: q3 + pi/2, d3: 0.00,
            a3: -0.054, alpha3: pi/2,   q4: q4,        d4: 1.50,
            a4: 0,      alpha4: pi/2,   q5: q5,        d5: 0.00,
            a5: 0,      alpha5: pi/2,   q6: q6,        d6: 0.00,
            a6: 0,      alpha6: 0,      q7: 0,         d7: 0.303
        }

        def homTransform(a, alpha, q, d):
            Hm = Matrix([[cos(q),                  -sin(q),            0,            a],
                          [sin(q)*cos(alpha),   cos(alpha)*cos(q),   -sin(alpha), -sin(alpha)*d],
                          [sin(alpha)*sin(q),   sin(alpha)*cos(q),   cos(alpha),  cos(alpha)*d],
                          [0,                           0,                   0,            1]])
            return Hm

        # Define Rotation Matrices around X, Y, and Z
        def Rot_X(q):
            R_x = Matrix([[1,    0,         0 ],
                        [0,   cos(q), -sin(q)],
                        [0,   sin(q),  cos(q)]
                        ])
            return R_x
        
        def Rot_Y(q):
            R_y = Matrix([[cos(q),    0,   sin(q)],
              [   0    ,   1 ,    0    ],
              [-sin(q) ,  0  , cos(q)]
             ])
            return R_y 
        
        def Rot_Z(q):
            R_z = Matrix([[cos(q),  -sin(q),   0],
              [sin(q) , cos(q)  ,  0],
              [  0      ,   0      , 0],
             ])
            return R_z

        def calculateJointAngles(R_EE, p_asMatrix, roll, pitch, yaw):
            #Compensate for Rotation discrepancy
            RotationError = Rot_Z(pi) * Rot_Y(-pi/2)
            R_EE = R_EE * RotationError
            R_EE = R_EE.subs({'r' : roll, 'p': pitch, 'y': yaw})
 
            #Wristcenter Wc :
            Wc = p_asMatrix - 0.303 * R_EE[:, 2]
            Wc_x = Wc[0]
            Wc_y = Wc[1]
            Wc_z = Wc[2]

            sqd = sqrt(Wc_x**2 + Wc_y**2)
            
            theta_1 = atan2(Wc[1], Wc[0])

            a = 1.501 # Found by using "measure" tool in RViz.
            b = sqrt(pow((sqd - 0.35), 2) + pow((Wc_z - 0.75), 2))
            c = 1.25 # Length of joint 1 to 2.

            alpha = acos((b*b + c*c - a*a) / (2*b*c))
            beta = acos((a*a + c*c - b*b) / (2*a*c))
            delta = atan2(Wc_z - 0.75, sqd - 0.35)
            theta2 = pi/2 - alpha - delta

            # Look at Z position of -0.054 in link 4 and use it to calculate epsilon
            epsilon = 0.036 
            theta3 = pi/2 - (beta + epsilon)
            return (R_EE, Wc, theta1, theta2, theta3)



        

        #Transformation matrices:
        T0_1 =  homTransform(a0, alpha0, q1, d1)
        T0_1 = T0_1.subs(dh)

        T1_2 =  homTransform(a1, alpha1, q2, d2)
        T1_2 = T1_2.subs(dh)

        T2_3 =  homTransform(a2, alpha2, q3, d3)
        T2_3 = T2_3.subs(dh)

        T3_4 =  homTransform(a3, alpha3, q4, d4)
        T3_4 = T3_4.subs(dh)

        T5_6 =  homTransform(a5, alpha5, q6, d6)
        T5_6 = T5_6.subs(dh)

        T6_G =  homTransform(a6, alpha6, q7, d7)
        T6_G = T6_G.subs(dh)
        
        #FInal TRansformation matrix from base to gripper..
        T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T5_6 * T6_G)
        
        #------------------------------------------------------------------------------------
        # Inverse Kinematics Part starts here.......
        #------------------------------------------------------------------------------------

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            
            p_asMatrix = Matrix([[px], [py], [pz]])

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            r, p, y = symbols('r, p, y')
            #Intrinsic rotation applied on end-effector.
            R_EE = Rot_Z(y) * Rot_Y(p) * Rot_X(r)

            R_EE, Wc, theta1, theta2, theta3 = calculateJointAngles(R_EE, p_asMatrix, roll, pitch, yaw)

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3 })

            R0_3_Tp = R0_3.T
            
            R3_6 = R0_3_Tp * R_EE

            theta6 = atan2(-R3_6[1,1], R3_6[1,0])# +0.45370228
            sq5 = -R3_6[1,1]/sin(theta6)
            cq5 = R3_6[1,2]
            theta5 = atan2(sq5, cq5)
            sq4 = R3_6[2,2]/sin(theta5)
            cq4 = -R3_6[0,2]/sin(theta5)
            theta4 = atan2(sq4, cq4)






            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    # 
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
        ###

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
