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
[image4]: ./misc_images/DH_parameter_diagram.JPG
[image5]: ./misc_images/theta1.JPG
[image6]: ./misc_images/theta2_theta3.JPG
[image7]: ./misc_images/theta3_correction.JPG
[image8]: ./misc_images/theta2_theta3_alternate_solution.JPG
[image9]: ./misc_images/crazy_trajectory3.JPG
[image10]: ./misc_images/demo_mode1.png
[image11]: ./misc_images/demo_mode4.png
[image12]: ./misc_images/IK2.png
[image13]: ./misc_images/IK5.png
[image14]: ./misc_images/IK10.png
[image15]: ./misc_images/IK13.png
[image16]: ./misc_images/IK17.png
[image17]: ./misc_images/IK18.png
[image18]: ./misc_images/IK22.png
[image19]: ./misc_images/FK_demo1.png
[image20]: ./misc_images/FK_demo2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

**Forward Kinematics Demo:**

`roslaunch kuka_arm forward_kinematics.launch`

I changed the joint angles to understand their positive direction of rotation (see figure below).

![alt text][image20]

`rosrun tf tf_echo base_link gripper_link`

Later, I verified this end-effector position with my forward kinematics code, which is explained in next topic. 

![alt text][image19]


In below image, I have defined DH frames for each joint as per the steps given in the lessons.
![alt text][image4]

Using the image above, I filled up the DH parameter table using [URDF file](./kuka_arm/urdf/kr210.urdf.xacro). I used the guideline in [as mentioned here](./DH_parameters) Below is the DH parameter table:

**i** | **alpha(i-1)** | **a(i-1)** | **d(i)** | **theta(i)**
--- | --- | --- | --- | ---
 1 | 0 | 0 | d1 | theta1
 2 | -pi/2 | a1 | 0 | theta2 - pi/2
 3 | 0 | a2 | 0 | theta3
 4 | -pi/2 | a3 | d4 | theta4
 5 | pi/2 | 0 | 0 | theta5
 6 | -pi/2 | 0 | 0 | theta6
 7 | 0 | 0 | dG | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I have written the [code for Forward Kinematics](./kuka_arm/scripts/kuka_arm_fw_kinematics.py) It generates all individual joint transformation matrices and also homogeneous transform between base_link and gripper_link. Uncomment the print statements at the end of the code as required.

The output matrices generated using the code above are [here](./FK_output.txt).

I also ran *forward_kinematics* in demo mode and verified *my forward kinematics implementation*.

*Note: For better visualization, use **Notepad++** for FK_output.txt .*


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

As the Z-axis of last three joints meet at one point (at joint J5), the kinematics problem is decoupled into Inverse Position Kinematics and Inverse Orientation Kinematics. Thus, the **first 3 joint angles** are responsible for the **position** of the end-effector and **last three joints** will be responsible for the **orientation** of the end-effector.
First, I Calculate the wrist center (WC) position. WC is the translation of the end-effector point P along the Z-axis of joint 6 frame.

Hence, **WC = P - R0_6 * [0, 0, dG]**.  Here, dG is the distance between WC and EE from URDF file.

Then I use WC to calculate theta1, theta2 and theta3, as WC doesn't change with change in theta4-6.
**Theta1** is calculated by projecting the WC on X-Y plane as *atan2(wcy,wcx)*. Refer to image below (images are not upto scale):

![alt text][image5]

For **theta2 and theta3**, I constructed the triangle with joint2, joint3 and WC and I imagine the arm in X-Z plane by setting theta1=0 as shown below -

![alt text][image6]

I imagined the same problem with theta1=0, so that I have to deal only with 2-D. Therefore, as theta1 is URDF Z-axis rotation, the value of Z remains same in 2-D frame. But value of X in this frame is different and is calculated below as wcx11.

*wcx11 = sqrt(wcx^2 + wcy^2)* .... The X-component when arm is represented in X-Z plane

Also, I shift the origin to joint2 and represent WC in that frame (wcx1,wcz1), as shown below.

*wcx1 = wcx11 - a1*

*wcz1 = wcz - d1*

The two sides of the triangle are link2 and link3, which I obtained from the [URDF file](./kuka_arm/urdf/kr210.urdf.xacro). The third side was simply the magnitude of WC in that frame *(sqrt(wcx1^2 + wcz1^2))*. Then I applied cosine rule of triangles to calculate q2 and q3, from which I further calculate joint angles theta2 and theta3 as below:

*`theta22 = acos((r1^2 + t1^2 - s1^2)/(2*r1*t1))`* 

*`theta33 = acos((r1^2 + s1^2 - t1^2)/(2*r1*s1))`*

*`angel_t1 = atan2(wcz1,wcx1)`*

*`theta2 = pi/2 - (theta22 + angle_t1)`*

*`theta3 = pi/2 - theta33`*

But the angle theta3 calculated above is not the actual joint angle(Refer figure below). Thus, a correction is needed to get the joint angle theta3. Refer the figure below. When joint angle theta3=0, the above calculated theta3 will be alpha. Therefore, we need to subtract this offset alpha from the calculated theta3.

*alpha = atan(b/a)* ... a and b are obtained from [URDF file](./kuka_arm/urdf/kr210.urdf.xacro).

a=d4 and b=a3 as in DH parameter table.

*`theta3 = pi/2 - theta33 - alpha`*


![alt text][image7]


**Considering other possibilities:**

**For theta1:**
Mathematically, theta2 and theta3 can also be found for theta1 = theta1 + pi. For this, imagine the arm to be flipped about the Z-axis of base, and theta2 and theta3 turning all the way up to reach the point behind. But that would mean larger joint angles, and hence more redundant motion. Also, the joint angles will be beyond their physical limitations resulting in link collisions. As there is a fixed link between joint1 and joint2 which also extends X-direction of the DH frame of joint1, in this flipped case, reach of end-effector will be reduced by this length.

**For theta2 and theta3:**
The possibilites for alternative theta2 and theta3 are as shown below -

![alt text][image8]

The main disadvantage in this case is when given end-effector position will be closer to the base link, the joint3 will hit the ground (as shown in (b) in figure above). Also, both joint angle magnitudes will be greater (as shown in (a) and (b) in figure above). Therefore, this solution was discarded.

**Calculating theta4 to theta6:**
As the joint angles **theta4 to theta6** only contribute to the rotation of the end effector and do not change WC position, instead of homogeneous transform matrices, I have only considered rotational matrices.
As taught in the lessons, R3_6 = transpose(R0_3) * R0_6.
I calculated R0_3 using the theta1,theta2,theta3 values. R0_6 is calculated from the roll,pitch,yaw values provided. Thus RHS is known. Using the matrices in forward kinematics, I independently calculated R3_6 symbolically.

**R3_6 =** 

Index | 0 | 1 | 2
--- | --- | --- | ---
0 | -sin(q4) * sin(q6) + cos(q4) * cos(q5) * cos(q6) | -sin(q4) * cos(q6) - sin(q6) * cos(q4) * cos(q5) | -sin(q5) * cos(q4)
1 | sin(q5) * cos(q6)| -sin(q5) * sin(q6) | cos(q5)
2 | -sin(q4) * cos(q5) * cos(q6) - sin(q6) * cos(q4) |  sin(q4) * sin(q6) * cos(q5) - cos(q4) * cos(q6) | sin(q4) * sin(q5)

Then I obtained the equations for theta4 to theta6 as below -

*`theta4 = atan2((-R3_6[2,2]), R3_6[0,2])`*

*`theta6 = atan2(R3_6[1,1], (-R3_6[1,0]))`*

*`theta5 = atan2((R3_6[1,0]/cos(theta6)), R3_6[1,2])`*

In case `cos(theta6)==0`, I calculate theta6 using sin(theta6) as below -

*`theta5 = atan2((-R3_6[1,1]/sin(theta6)), R3_6[1,2])`*

*Note: theta4 can also be used to calculate theta5.* 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

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
3. Wrist Center (WC)  is calculated as P - R0_6 * [0, 0, dG]. Here, dG is the distance between WC and Gripper from URDF file.
5. angle_t1 is the angle of WC w.r.t. the positive X-axis in the new frame wehre joint2 is the origin.
4. theta22 and theta33 are the angles as shown above, calculated using the cosine rule of triangles
5. theta2 and theta3 are calculated from theta22 and theta33 respectively. (refer figure above)
6. R0_3 is calculated from substituting first three joint angle values in R0_3_sym.
7. theta4 and theta6 are directly calculated as they are independently represented by atan2.
8. theta5 cannot be independently represented in atan2. Without atan2, there would be ambiguity in value and sign of theta5. It could either be calculated from  theta4 or theta6. I chose theta6. But the first calculation of theta5 contains cos(theta6) in denominator. So for theta6 very close to pi/2, the value will be incorrect. In that case, the next 'if' condition will become true, and there I use sin(theta6) to instead of cos(theta6).

**Results:**
The end-effector moved the `exact same trajectory` as provided(see below). 

![alt text][image12]

![alt text][image13]

![alt text][image14]


**And Even in case of Wierd Trajectories!!**

![alt text][image9]

I was able to `successfully perform the pick and place for all object positions`. 

![alt text][image15]

![alt text][image16]

![alt text][image17]

![alt text][image18]



When *theta5 is zero*, there are two possible solutions for theta4 and theta6, as their axis of rotations will match. I haven't tackled this condition explicitly. Therefore, the arm performs some unnecessary rotation of joint4 and joint6 at the beginning. 



