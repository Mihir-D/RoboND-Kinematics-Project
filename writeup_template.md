## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Overview of the Logic to calculate joint angles:
Calculate the wrist center (WC) position. Use WC to calculate theta1, theta2 and theta3, as WC doesn't change with change in theta4-6.
Theta1 is calculated by projecting the WC on X-Y plane as atan2(wcy,wcx).
For theta2 and theta3, I constructed the triangle with joint2, joint3 and WC and I project the arm in X-Z plane by setting theta1=0 as shown below.- ??? I imagined the same problem with theta1=0, so that i have to deal only with 2-D.
I shift the origin to joint2 and represent WC in that frame (wcx1,wcz1). The two sides of the triangle are link2 and link3, which I obtained from the URDF file (put link here???). The third side was simply the magnitude of WC in that frame (sqrt(wcx1^2 + wcz1^2)). Then I applied cosine rule of triangles to calculate q2 and q3, from which I further calculate joint angles theta2 and theta3.
As the joint angles theta4 to theta6 only contribute to the rotation of the end effector and do not change WC position, instead of homogeneous transform matrices, I have only considered rotational matrices.
As taught in the lessons, R3_6 = transpose(R0_3) * R0_6.
I calculated R0_3 using the theta1,theta2,theta3 values. R0_6 is calculated from the roll,pitch,yaw values provided. Thus RHS is known. Using the matrices in forward kinematics, I independently calculated R3_6 symbolically???. Then I obtained the equations for theta4 to theta6 using atan2() function. 

The workflow of the code:
Note: Sequence in which joint angles are calculated is - Theta1, Theta2, Theta3, Theta4, Theta6, Theta5
Once the EE(End Effector) positions are received, following steps are done only once, as they are common for all positions -
1. DH parameter symbols are created
2. DH parameters are defined and stored in a hash table
3. Symbols for roll, pitch and yaw are created
4. Next, R0_3 is symbolically calculated using the joint angle symbols q1, q2 and q3
5. A rotational matrix for correction between the DH frame and URDF frame of EE is calculated
6. Rotations about independent axis R_x1, R_y1 and R_z1 are symbolically defined
7. Euler Extrinsic rotation X-Y-Z is calculated symbolically and stored in "R0_6_sym". The multiplication by transpose of correction matrix accounts for R0_6 rotation from URDF frame to DH frame of reference.

Now, following steps are done for each position:
1. Get positional (x,y,z) and rotational (roll,pitch,yaw) vectors from the sent request
2. Evaluate the symbolic rotational matrix R0_6_sym to get the rotational matrix R0_6 in DH frame.
3. Wrist Center (WC)  is calculated as P - R0_6 * [0, 0, d4]. Here, d4 is the distance between WC and EE from URDF file.
5. angle_t1 is the angle of WC w.r.t. the positive X-axis in the new frame wehre joint2 is the origin.
4. theta22 and theta33 are the angles as shown above, calculated using the cosine rule of triangles
5. theta2 and theta3 are calculated from theta22 and theta33 respectively. (refer figure above)
6. R0_3 is calculated from substituting first three joint angle values in R0_3_sym.
7. theta4 and theta6 are directly calculated as they are independently represented by atan2.
8. theta5 cannot be independently represented in atan2. Without atan2, there would be ambiguity in value and sign of theta5. It could either be calculated from  theta4 or theta6. I chose theta6. But the first calculation of theta5 contains cos(theta6) in denominator. So for theta6 very close to pi/2, the value will be incorrect. In that case, the next 'if' condition will become true, and there I use sin(theta6).

And just for fun, another example image:
![alt text][image3]


