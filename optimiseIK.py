from mpmath import *
from sympy import *
import numpy as np

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  #Theta
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  #d-offset

a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  #
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # alpha twist angle..

dh = {
            a0: 0,      alpha0: 0,       q1: q1,        d1: 0.75,
            a1: 0.35,   alpha1: -pi/2,   q2: q2 - pi/2, d2: 0.00,
            a2: 1.25,   alpha2: 0,       q3: q3,        d3: 0.00,
            a3: -0.054, alpha3: -pi/2,   q4: q4,        d4: 1.50,
            a4: 0,      alpha4: pi/2,    q5: q5,        d5: 0.00,
            a5: 0,      alpha5: -pi/2,   q6: q6,        d6: 0.00,
            a6: 0,      alpha6: 0,       q7: 0,         d7: 0.303
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

def calculateJointAngles(Wc):
            
    Wc_x = Wc[0]
    Wc_y = Wc[1]
    Wc_z = Wc[2]

    sqd = sqrt(Wc_x**2 + Wc_y**2)
            
    theta1 = atan2(Wc[1], Wc[0])

    a = 1.501 
    b = sqrt(pow((sqd - 0.35), 2) + pow((Wc_z - 0.75), 2))
    c = 1.25 

    angle_a = acos((b*b + c*c - a*a) / (2*b*c))
    angle_b = acos((a*a + c*c - b*b) / (2*a*c))
    angle_c = acos((a*a - c*c + b*b) / (2*a*b))

    delta =  atan2( Wc_z - 0.75, sqd - 0.35 )
    theta2 = pi/2 - (angle_a + delta) 

    theta3 = pi/2 - (angle_b + 0.036)
            
    return (theta1, theta2, theta3)


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


def obtainJoints(req):
    #End Effector Position
    '''px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z'''

    (px, py, pz) = (2.16135,-1.42635,1.55109)

    EE_Matrix = Matrix([[px], [py], [pz]])

    (roll, pitch, yaw) = (1.6544359732979843, 0.4899095071534359, 0.062392126062629866)

    #End Effector Orientation angles
    '''(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])'''

    r, p, y = symbols('r, p, y')
            #Intrinsic rotation applied on end-effector.
    R_EE = Rot_Z(y) * Rot_Y(p) * Rot_X(r)
            #Rotation Error
    RotationError = Rot_Z(pi) * Rot_Y(-pi/2)
    R_EE = R_EE * RotationError
            # Substitute the End Effector Orientation angles for r, p, y
    #R_EE = R_EE.subs({'r' : roll, 'p': pitch, 'y': yaw})

    R_EE = R_EE.evalf(subs = {'r' : roll, 'p': pitch, 'y': yaw})

            #Wrist Center Position
    Wc = EE_Matrix - 0.303 * R_EE[:, 2]
            #Compute the Joint angles 1,2 & 3 from wrist center positions
    theta1, theta2, theta3 = calculateJointAngles(Wc)

            # Evaluate the Rotation Matrix from {0} to {3} with the obtained 
            # theta1, theta2 & theta3 values.

    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3 })

    R0_3_Tp = R0_3.T

    # As we know that R_EE = R0_3 * R3_6 and inv(R0_3) = Transpose(R3_6) we can write,
            
    R3_6 = R0_3_Tp * R_EE

            # Now that we know the Rotation matrix from {3} to {6} and the 
            # End Effector Orientation at {6}. So from R3_6, Euler angles can be extracted 
            # and equalled with obtained roll, pitch and yaw angles.
    
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2( sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2] )
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    joints_calc = (theta1, theta2, theta3, theta4, theta4, theta6)

    #T0_G = T0_G.evalf(subs = {q1: joints_calc[0], q2: joints_calc[1], q3: joints_calc[2], q4: joints_calc[3], q5: joints_calc[4], q6: joints_calc[5] })


    return joints_calc

def forwardKinematics(joints):

    global T0_G

    T0_G = T0_G.evalf(subs = {q1: joints[0], q2: joints[1], q3: joints[2], q4: joints[3], q5: joints[4], q6: joints[5] })
    print(T0_G)
    
    wc = 0
    ef = 0
    return wc, ef

