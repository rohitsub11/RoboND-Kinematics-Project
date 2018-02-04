## Project: Kinematics Pick & Place
---
[//]: # (Image References)
[start]: ./readme_images/start.jpg
[dh]: ./readme_images/dh.png

![Start][start]

In this project, we are working with a simulation of Kuka KR210 to pick up cans from a shelf and then put them in a box.

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
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

The homogeneous transform for the DH coordinates system from joint `i-1` to `i` is:

Let `T_i-1,i = `

               [[cos(theta_i), -sin(theta_i), 0, a_i-1], 
               [sin(theta_i) * cos(alpha_i-1), cos(theta_i) * cos(alpha_i-1), -sin(alpha_i-1), -sin(alpha_i-1) * d_i-1], 
               [sin(theta_i) * sin(alpha_i-1), cos(theta_i) * sin(alpha_i-1), cos(alpha_i-1), cos(alpha_i-1) * d_i-1], 
               [0, 0, 0, 1]]
               
We fill in the `alpha`, `a`, and `d` values from above table and must later calculate `theta` values.
The overall homogeneous transform from base frame to end-effector frame is them:               
 
              
`T_0EE = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6EE`

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
        
`Rot_EE = rotZ * rotY * rotX`

To compare results with Rviz, we must rotate the R0_G frame. That correction is given by:

`Rot_Error = rotZ(y=pi) * rotY(p=-pi/2)`

`Rot_EE = Rot_EE * Rot_Error = f(r, p, y)`

### Inverse Kinematics

We have a spherical wrists robot with joint 5 being the *wrist center*.  Thus
we can kinematically decouple the IK problem into *Inverse Position* and *Inverse Orientation*

### Inverse Position

Wrist center position determined by first three joints. Use complete transform matrix based on 
end-effector pose.

`WC = [[px], [py], [pz]] - 0.303 * Rot_EE[:, 2]`

`theta1 = arctan2(WC_y, WC_x)`

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

`r03 = t01[0:3, 0:3] * t12[0:3, 0:3] * t23[0:3, 0:3]`

`r03 = r03(q1=theta1, q2=theta2, q3=theta3)`

Multiplying both sides of the above equation by `LU_inverse(R0_3)`
Calculate inverse using Sympy's `inv()` passing "LU" ensuring "LU decomposition."
`r36 = LU_inverse(r03) * Rot_EE`

Resultant matrix on the RHS does not have any variables after substituting joint angle values.

*Compute Euler angles from the rotation matrix from 3-6*

`theta4 = atan2(r36[2, 2], -r36[0, 2])`

`theta5 = atan2(sqrt(r36[0, 2]^2 + r36[2, 2]^2), r36[1, 2])`

`theta6 = atan2(-r36[1, 1], r36[1, 0])`

`joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]`


