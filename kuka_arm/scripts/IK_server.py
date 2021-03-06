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
import numpy as np
import time

# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
r, p, y = symbols("r p y") # end-effector orientation
px_s, py_s, pz_s = symbols("px_s py_s pz_s") # end-effector position

def rot_x(q):
    #rotation matrix along x axis
    R_x = Matrix([[      1,      0,      0],
                  [      0, cos(q), -sin(q)],
                  [      0, sin(q),  cos(q)]])
    return R_x

def rot_y(q):
    #rotation matrix along y axis
    R_y = Matrix([[ cos(q),     0, sin(q)],
                  [      0,     1,      0],
                  [-sin(q),     0, cos(q)]])
    return R_y

def rot_z(q):
    #rotation matrix along z axis
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q),  cos(q), 0],
                  [     0,       0, 1]])
    return R_z

def DH_transform(q, d, alpha, a):
    #alpha: twist angle, a: link length
    #d: link offset, q: joint angle
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    return T

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Compensation matix from URDF to world frame
        R_corr = rot_z(pi) * rot_y(-pi/2)

        theta1,theta2,theta3,theta4,theta5,theta6 = 0,0,0,0,0,0
        angles_pre = (0,0,0,0,0,0)
        r2d = 180./np.pi
        loop_times = []
        # Initialize service response
        joint_trajectory_list = []
        #
        # Create Modified DH parameters
        s = {alpha0:       0, a0:      0, d1:  0.75,
             alpha1:   -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
             alpha2:       0, a2:   1.25, d3:     0,
             alpha3:   -pi/2, a3: -0.054, d4:   1.5,
             alpha4:    pi/2, a4:      0, d5:     0,
             alpha5:   -pi/2, a5:      0, d6:     0,
             alpha6:       0, a6:      0, d7: 0.303, q7: 0}
        # Define Modified DH Transformation matrix
        # Create individual transformation matrices
        # base_link to link1
        T0_1 = DH_transform(q1, d1, alpha0, a0).subs(s)
        # linke1 to link 2
        T1_2 = DH_transform(q2, d2, alpha1, a1).subs(s)
        # link2 to link3
        T2_3 = DH_transform(q3, d3, alpha2, a2).subs(s)
        # link3 to link4
        T3_4 = DH_transform(q4, d4, alpha3, a3).subs(s)
        # link4 to link5
        T4_5 = DH_transform(q5, d5, alpha4, a4).subs(s)
        # link5 to link6
        T5_6 = DH_transform(q6, d6, alpha5, a5).subs(s)
        # link6 to end-effector
        T6_EE = DH_transform(q7, d7, alpha6, a6).subs(s)

        #from base to end-effector
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
    

        # Extract rotation matrices from the transformation matrices

        #
        #
        ###
        loop_start_time = time.time()
        for x in xrange(0, len(req.poses)):
            loop_current_time = time.time()
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            Rot_EE = rot_z(y) * rot_y(p) * rot_x(r)  # Roll pitch yaw
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rot_Error = rot_z(y).subs(y, radians(180)) * rot_y(p).subs(p, radians(-90))

            Rot_EE = Rot_EE*Rot_Error

            Rot_EE =  Rot_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

            EE = Matrix([[px],
                         [py],
                         [pz]])

            # calculate wrist center position from end-effector position!
            # formula: 0_r_WC/0 = 0_r_EE/0 - d * 0-6R * [[0],[0][1]] -
            # [[px], [py], [pz]] - d * 0-6R [[0][0][1]]
            WC = EE - (0.303) * Rot_EE[:,2]
    	    #
    	    # Calculate joint angles using Geometric IK method
    	    theta1 = atan2(WC[1],WC[0])

            # SSS triangle for theta2 and 3
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25

            angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2 * side_b *side_c))
            angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2 * side_a *side_c))
            angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2 * side_a *side_b))

            theta2 = pi/2-angle_a-atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            # constant account for sag in link4 of -0.054m
            theta3 = pi/2-(angle_b+0.036)

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

            R3_6 = R0_3.inv("LU")*Rot_EE


            #Euler angles from rot matrix
            theta4=atan2(R3_6[2,2], -R3_6[0,2])
            theta5=atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6=atan2(-R3_6[1,1], -R3_6[1,0])

        
            ###
            loop_time = time.time()-loop_current_time
            loop_times.append(loop_time)
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
        print("Inverse kinematics calculation has been done!")
        print("Total calculation time: {:>4f}".format(time.time() - loop_start_time))
        print("Average calculation time: {:>4f}".format(np.mean(loop_times)))
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        print("-----------------------------------------")
        print("Moving robot arm...")
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
