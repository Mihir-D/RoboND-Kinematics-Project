#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        # IK code starts here
	#print("Mihir1")

        # Define DH param symbols
        #Creating symbols for joint variables
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	#Defining DH parameters
	### Kuka KR210 ###
	s_ = {alpha0:       0,  a0:       0, d1:    0.75,
	     alpha1:  -pi/2.,  a1:    0.35, d2:       0, q2: q2 - pi/2.,
	     alpha2:       0,  a2:    1.25, d3:       0,
	     alpha3:  -pi/2.,  a3:  -0.054, d4:    1.50,
	     alpha4:   pi/2.,  a4:       0, d5:       0,
	     alpha5:  -pi/2.,  a5:       0, d6:       0,
	     alpha6:       0,  a6:       0, d7:   0.303, q7: 0}


            
        # Joint angle symbols
	roll1 = symbols('roll1')
	pitch1 = symbols('pitch1')
	yaw1 = symbols('yaw1')

      
        # Modified DH params


        # Define Modified DH Transformation matrix
	#Homogeneous transforms
        R0_1 = Matrix([[           cos(q1),            -sin(q1),            0],
                     [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0)],
                     [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0)]])
        R0_1 = R0_1.subs(s_)

        R1_2 = Matrix([[           cos(q2),            -sin(q2),            0],
                     [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1)],
                     [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1)]])
        R1_2 = R1_2.subs(s_)

        R2_3 = Matrix([[           cos(q3),            -sin(q3),            0],
                     [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2)],
                     [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2)]])
        R2_3 = R2_3.subs(s_)

        # Create individual transformation matrices
	#Composition of Homogeneous Transforms
        R0_2 = simplify(R0_1 * R1_2)
        R0_3_sym = simplify(R0_2 * R2_3)

	#Correction needed to account of orientation difference between definition of gripper link
	# between URDF file vs. DH convention
        R_y = Matrix([[ cos(-pi/2),        0, sin(-pi/2)],
                      [ 0,                 1,         0],
                      [-sin(-pi/2),        0, cos(-pi/2)]])
        R_z = Matrix([[ cos(pi), -sin(pi),       0],
                      [ sin(pi),  cos(pi),       0],
                      [       0,        0,       1]])
        R_corr = simplify(R_z * R_y)

        #T_total =  simplify(T0_G * R_corr)
	#print("Mihir2")

        #Using Rrpy to calculate R0_6
	R_x1 = Matrix([[ 1,              0,        0],
                      [ 0,        cos(roll1), -sin(roll1)],
                      [ 0,        sin(roll1),  cos(roll1)]])

        R_y1 = Matrix([[ cos(pitch1),        0,  sin(pitch1)],
                      [       0,        1,        0],
                      [-sin(pitch1),        0,  cos(pitch1)]])

        R_z1 = Matrix([[ cos(yaw1), -sin(yaw1),        0],
                      [ sin(yaw1),  cos(yaw1),        0],
                      [ 0,              0,        1]])
        #Below is extrinsic Z-Y-X rotation
        #R0_6_sym = simplify(R_x1 * R_y1 * R_z1 * R_corr.transpose())
        R0_6_sym = simplify(R_z1 * R_y1 * R_x1 * R_corr.transpose()) #extrinsic X-Y-Z
	#print("Mihir3")

        for x in xrange(0, len(req.poses)):
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
            #print(roll, "  ",  pitch, "  ", yaw)
     
            # Calculate joint angles using Geometric IK method
	    R0_6 = R0_6_sym.evalf(subs={roll1: roll, pitch1: pitch, yaw1:yaw})
	    P = Matrix([px, py, pz]) 
	    M1 = Matrix([0, 0, 0.303]) # 0.303 + 0.15(gripper length) = 0.453
	    WC = P - R0_6*M1 
	    wcx = WC[0]
	    wcy = WC[1]
            wcz = WC[2]
            #print("Mihir5")
	    theta1 = atan2(wcy, wcx)
	    #print("theta1", theta1)
	    wcx11 = sqrt(wcx**2 + wcy**2) #transfering WC to X-Z plane
	    wcx1 = wcx11 - 0.35 # wcx - a1, -0.15 is gripper length
	    wcz1 = wcz - 0.75 #+ 0.2 # wcz - d1
            #print("Mihir6")
	    a11 = wcx1
	    b11 = wcz1
	    r1 = 1.25
            s1 = 1.501 #sqrt(d4**2 + a3**2)
            t1 = sqrt(wcx1**2 + wcz1**2)
            c11 = (r1**2 + (wcx1)**2 + (wcz1)**2 - s1**2)/(2*r1)
	    #below line is tested
            #theta2 = pi/2 - (atan2(b11,a11) + acos(c11/(t1)))
	    """
            r1 = 1.25	#link2 length
            s1 = 1.501 #link3 length till Wrist Center (WC)
	    cosq1_q2 = (wcx1**2 + wcz1**2 - r1**2 - s1)/(2*r1*s1)
	    sinq1_q2 = sqrt(1 - cosq1_q2 ** 2)
	    t1 = sqrt(wcx1**2 + wcz1**2)
	    angle_t1 = atan2(wcz1, wcx1)
	    theta2 = (angle_t1 + atan2(2*s1*sinq1_q2*r1, r1**2 + t1**2 - s1**2))
	    theta2 -= pi/2
	    """
            angle_t1 = atan(wcz1/wcx1)
	    theta22 = acos((r1**2 + t1**2 - s1**2)/(2*r1*t1))
	    #(pi/2 - theta33) is the angle of hypothetical link between joint 3 and WC, w.r.t horizontal X axis. Rotation upwards being negative
	    #Hence, to know joint angle theta3, a correction is needed as shown below
	    theta33 = acos((r1**2 + s1**2 - t1**2)/(2*r1*s1))
	    #Convert theta33 to joint angle theta3
	    #J3_5 = 1.5 (d4), J3_4 = 0.96, height = 0.054 (a3)
	    theta3_corr = 0.036  #0.0562 #0.020
	    theta2 = pi/2 - (angle_t1 + theta22)
	    theta3 = (pi/2 - theta33 - theta3_corr)
	    #print("theta2: ", theta2)
            #print("theta3: ", theta3)

	    ### calculating theta4, theta5, theta6
	    R0_3 = R0_3_sym.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
	    R3_6 = simplify(R0_3.transpose() * R0_6)
	    #below equations are calculated from "Forward Kinematics" 
	    theta4 = atan2((-R3_6[2,2]), R3_6[0,2]) 
	    theta6 = atan2(R3_6[1,1], (-R3_6[1,0]))
	    theta5 = atan2((R3_6[1,0]/cos(theta6)), R3_6[1,2])
	    #In case cos(theta6)==0, calculate theta6 using sin(theta6)
	    if abs(abs(theta6) - pi/2) < 0.00001:
		theta5 = atan2((-R3_6[1,1]/sin(theta6)), R3_6[1,2])
	    #elif theta5 < -2:
	    #    theta5 = atan2((-R3_6[1,1]/sin(theta6)), R3_6[1,2])
            #print("theta4= ", theta4)
            #print("theta5= ", theta5)
            #print("theta6= ", theta6)
	    #theta1 = pi/4
            #theta2 = 0
            #theta3 = 0
            #theta4 = 0
            #theta5 = pi/2
            #theta6 = 0

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
