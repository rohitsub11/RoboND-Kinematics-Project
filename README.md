## Project: Kinematics Pick & Place
---
[//]: # (Image References)
[start]: ./readme_images/start.jpg
[dh]: ./readme_images/dh.png
[missed]: ./readme_images/missed_cylinder.jpg
[final]: ./readme_images/in_bin.jpg

![Start][start]

In this project, we are working with a simulation of Kuka KR210 to pick up cans from a shelf and then put them in a box.

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
Forward Kinematics (FK) is a set of methods to calculate the final coordinate position and rotation of end-effector of a conjoined links (e.g. robotic arms, limbs, etc.), given parameters of each joint between links. In this project, these parameters are angles of each joint, totalling 6 joints (i.e. 6 Degrees of Freedom).

Inverse Kinematics (IK), on the other hand, is the exact opposite of FK, where we calculate the parameters from a given coordinate position and rotation.

# Homogenous Transforms

To calculate FK and IK calculation, we attach reference frames to each link of the manipulator and writing the homogeneous transforms from the fixed base link to link 1, link 1 to link 2, and so forth, all the way to the end-effector.

To do FK and IK, we are using a method by Jacques Denavit and Richard Hartenberg which requires only four parameters for each reference frame.

![dh][dh]
Definition of DH parameters
- Twist angle (alpha) is the angle between `z_i-1` and `z_i` as measured about `x_i-1` in the right-hand sense
- Link length (a) is the distance between `z_i-1` and `z_i` along `x_i-1` where `x_i-1` is perpendicular to both `z_i-1` and `zi`
- Link offset (d) is the signed distance from `x_i-1` to `x_i` measure along `z_i`. Will be a variable for a prismatic joint.
- Joint angle (theta) is the angle between `x_i-1` to `x_i` measured about `z_i` in the right-hand send. Will be a variable for a revolute joint.

From URDF file:
- J0 = (0, 0, 0)
- J1 = (0, 0, 0.33)
- J2 = (0.35, 0, 0.42)
- J3 = (0, 0, 1.25)
- J4 = (0.96, 0, -0.054)
- J5 = (0.54, 0, 0)
- J6 = (0.193, 0, 0)
- JG = (0.11, 0, 0)

Following are DH parameters used specifically in this project:

Links | theta(i) | d(i-1) | alpha(i-1) | a(i-1)
--- | --- | --- | --- | ---
0->1 | 0 | 0.75 | 0 | 0
1->2 | q2 - pi/2 | 0 | -pi/2 | 0.35
2->3 | 0 | 0 | 0 | 1.25
3->4 |  0 | 1.5 | -pi/2 | -0.054
4->5 | 0 | 0 | pi/2 | 0
5->6 | 0 | 0 | -pi/2 | 0
6->EE | 0 | 0.303 | 0 | 0

- `a1` is x-axis distance beteen `z1` and `z2`. in URDF joint 1 and joint 2 are 0.35m apart
- `d1` is distance between `x0` and `x1` along `z1`, which is 0.33 + 0.42 = 0.75m

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The homogeneous transform for the DH coordinates system from joint `i-1` to `i` is:

Let `T_i-1,i = `

               [[cos(theta_i), -sin(theta_i), 0, a_i-1], 
               [sin(theta_i) * cos(alpha_i-1), cos(theta_i) * cos(alpha_i-1), -sin(alpha_i-1), -sin(alpha_i-1) * d_i-1], 
               [sin(theta_i) * sin(alpha_i-1), cos(theta_i) * sin(alpha_i-1), cos(alpha_i-1), cos(alpha_i-1) * d_i-1], 
               [0, 0, 0, 1]]
               
We fill in the `alpha`, `a`, and `d` values from above table and must later calculate `theta` values.

We'll create a method for the generalized transform matrix:
      ```
        def homogeneous_transform(alpha, a, d, q):
            return Matrix([[           cos(q),            -sin(q),           0,             a],
                           [sin(q)*cos(alpha),  cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                           [sin(q)*sin(alpha), cos(q)*sin(allpha),  cos(alpha),  cos(alpha)*d],
                           [                0,                  0,           0,             1]])
      ```

We can now create our individual transforms by calling this method and using the DH dictionary created and substituting
  it in:
      ```
        T0_1  = homogeneous_transform(alpha0, a0, d1, q1).subs(s)
        T1_2  = homogeneous_transform(alpha1, a1, d2, q2).subs(s)
        T2_3  = homogeneous_transform(alpha2, a2, d3, q3).subs(s)
        T3_4  = homogeneous_transform(alpha3, a3, d4, q4).subs(s)
        T4_5  = homogeneous_transform(alpha4, a4, d5, q5).subs(s)
        T5_6  = homogeneous_transform(alpha5, a5, d6, q6).subs(s)
        T6_EE = homogeneous_transform(alpha6, a6, d7, q7).subs(s)
      ```

The overall homogeneous transform from base frame to end-effector frame is them:               
 
              
`T_0EE = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6EE`

But wait, there's more. We still need to account for the difference in orientation of the gripper link as described in the URDF vs the DH convention. 
  
     * We first apply a body fixed rotation about the Z axis
     * Then about the Y axis and x axis.

*Let `r=rotation`, `p=pitch`, `y=yaw`, then the x, y, and z rotations in a 3D space are as follows:*

`rotX = [[1, 0, 0],
        [0, cos(r), -sin(r)],
        [0, sin(r), cos(r)]]`        
`rotY = [[cos(p), 0, sin(p)],
        [0, 1, 0],
        [-sin(p), 0, cos(p)]]`        
`rotZ = [[cos(y), -sin(y), 0],
        [sin(y), cos(y), 0],
        [0, 0, 1]]`

* We'll assign the composition of the these three rotations to a variable and return the corrected matrix
        
`Rot_EE = rotZ * rotY * rotX`

To compare results with Rviz, we must rotate the R0_G frame. That correction is given by:

`Rot_Error = rotZ(y=pi) * rotY(p=-pi/2)`

`Rot_EE = Rot_EE * Rot_Error = f(r, p, y)`

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

For the IK problem, our goal is to calculate the joint angles of the serial manipulator. We'll use an analytical or closed-form apporach since it's much faster than numerical approaches and is easier to develop rules for which of the possible solutions is the appropriate one.

* Before we do that however, this appoach requires the manipulator to meet one of the following criteria:
    1. 3 neighboring joint axes intersect at a single point
    2. 3 neighboring joint axes are parllel (this is a special case of the first criteria since parallel lines intersect at infinity)
* Fortunately our Kuka arm meets the first criteria for using this approach
 
Most of the time, the last 3 joints of a manipulator are revolute joints, as in the case of the Kuka arm, which is known as a **spherical wrist**. The common point of intersection of the spherical wrist is known as the wrist center (WC). This design
kinematically decouples the position and orientation of the EE and thus simplifying the IK problem into 2:

1. The cartesian coordinates of the wrist center (WC)
2. The composition of rotations to orient the end effector (EE)

In a 6-DOF manipulator with a spherical wrist, it uses the first 3 joints to control the position (Joints 1, 2, and 3) of the WC and the last 3 (Joints 4, 5, and 6) to orient the EE.


### Inverse Position
1. #####Solving for WC

Wrist center position determined by first three joints. Use complete transform matrix based on 
end-effector pose.

`WC = [[px], [py], [pz]] - 0.303 * Rot_EE[:, 2]`

Now that we have the WC position, we can now derive for the equations for the first three joints.

2. #####Solving for Joint 1, 2, and 3
    a. **Joint 1**
	
    	* Solving joint 1 is quite simple, we just need to project WC's Z axis onto the ground plane

			`theta1 = arctan2(WC_y, WC_x)`
	 b. **Joint 2 and Joint 3**
	
    	* As mentioned in lecture, theta2 and theta3 are tricky to visualize.

			`side_a = 1.501`

			`side_b = sqrt((sqrt(WC[0]^2 + WC[1]^2) - 0.35)^2 + (WC[2] - 0.75)^2)`

			`side_c = 1.25`

			`angle_a = acos((side_b^2 + side_c^2 - side_a^2) / (2 * side_b * side_c))`
			                
			`angle_b = acos((side_a^2 + side_c^2 - side_b^2) / (2 * side_a * side_c))`

			`angle_c = acos((side_a^2 + side_b^2 - side_c^2) / (2 * side_a * side_b))`

			`theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]^2 + WC[1]^2) - 0.35)`

			`theta3 = pi/2 - (angle_b + 0.036)` (constant accounts for sag in link4 of -0.054m)

### Inverse Orientation

Resultant transform give by:

`R0_6 = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6`

Overall roll, pitch, yaw rotation from base to gripper must equal product of 
individual rotations between respective links, the following holds:

`R0_6 = Rrpy`

where `Rrpy = homogeneous RPY rotation between base link gripper link`
3. ##### Solve for Joints 4, 5, and 6
    * Since we already calculated our individual transforms previously in the FK section, we can use that to solve for R0_3

		`r03 = t01[0:3, 0:3] * t12[0:3, 0:3] * t23[0:3, 0:3]`

		`r03 = r03(q1=theta1, q2=theta2, q3=theta3)`
	* Now we need the rotations from R3_6, to do that recall that since the overall RPY (Roll Pitch Yaw) rotation between base link and gripper link must be equal to the product of individual rotations between respective links, following holds true:
    	![rotation_equality](kinematic_analysis_images/rotation_equality.png)     
     
	* We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by inv(R0_3) which leads to:

		Multiplying both sides of the above equation by `LU_inverse(R0_3)`
		Calculate inverse using Sympy's `inv()` passing "LU" ensuring "LU decomposition."
		`r36 = LU_inverse(r03) * Rot_EE`

	* The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for joint 4, 5, and 6.

		*Compute Euler angles from the rotation matrix from 3-6*

		`theta4 = atan2(r36[2, 2], -r36[0, 2])`

		`theta5 = atan2(sqrt(r36[0, 2]^2 + r36[2, 2]^2), r36[1, 2])`

		`theta6 = atan2(-r36[1, 1], r36[1, 0])`

		`joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]`
4. ##### Now we choose the correct solution among the set of possible solutions

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
After having some help implementing IK, the script ran very slow due to its resource heavy calculations. So to optimize the performance I followed the recommended optimizations from lecture. I left the FK implementation and generic formulas outside the for loop since this doesn't need to be recomputed inside the for loop.

Although the IK was still slow, the arm managed to work without any issues with the pick and place cycles. The only problem besides the arm being slow was that the gripper wouldn't grip the cylinder fully when I click 'continue', so when it performs the IK part, there would be no cylinder in the robot's gripper but it would perform the cycle as we can see in the image below:

![missed_cylinder][missed]    


I took care of this issue by adding this line of code:

```
	ros::Duration(2.0).sleep();
```

in the src/trajectory_sampler.cpp, as suggested in the Common Questions section of the project which made it possible to grasp more cylinders as we can see below, the arm managed to place 3 cylinders in a bin.

![three_in_a_bin][final]    

Some improvements I would do after implementing the IK script, are simplifying the math formulas. There may be ways to solve IK using less expensive operators that I may not aware of, but the math operations behind this project seems to be the main choke point since it performs very heavy computations. Another work around is having a better machine that can handle these kind of computations.



