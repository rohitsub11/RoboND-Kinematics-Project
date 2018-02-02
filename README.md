## Project: Kinematics Pick & Place
---
[//]: # (Image References)
[start]: ./readme_images/start.jpg
[dh]: ./readme_images/dh.png
[alpha]: ./readme_images/alpha.png
[alpha_i-1]: ./readme_images/alpha_i-1.png
[a]: ./readme_images/a_i-1.png
[d]: ./readme_images/d_i.png
[theta]: ./readme_images/theta_i.png
[pi2]: ./readme_images/pi2.png
[-pi2]: ./readme_images/-pi2.png
[theta1]: ./readme_images/theta_1.png
[theta2]: ./readme_images/theta_2.png
[theta2-90]: ./readme_images/theta_2-90.png
[theta3]: ./readme_images/theta_3.png
[theta4]: ./readme_images/theta_4.png
[theta5]: ./readme_images/theta_5.png
[theta6]: ./readme_images/theta_6.png
[transform-single]: ./readme_images/transform-single.png
[transform-simple]: ./readme_images/transform-simple.png
[transform-composition1]: ./readme_images/transform-composition1.png
[transform-composition2]: ./readme_images/transform-composition2.png
[A_r_P_A_0]: ./readme_images/A_r_P_A_0.png
[A]: ./readme_images/A.png
[P]: ./readme_images/P.png
[A_0]: ./readme_images/A_0.png
[R_small]: ./readme_images/r.png
[r_11]: ./readme_images/r_11.png
[A_B_R]: ./readme_images/A_B_R.png
[rotation-single]: ./readme_images/rotation-single.png
[transform-comb]: ./readme_images/transform-comb.png
[diag-clean]: ./readme_images/diag-clean.png
[diag-detailed]: ./readme_images/diag-detailed.png
[O_0]: ./readme_images/O_0.png
[O_1]: ./readme_images/O_1.png
[O_2]: ./readme_images/O_2.png
[O_2_1]: ./readme_images/O_2_1.png
[O_EE]: ./readme_images/O_EE.png
[Z_1]: ./readme_images/Z_1.png
[theta_2-calc]: ./readme_images/theta_2-calc.png
[theta_2-zoom]: ./readme_images/theta_2-zoom.png
[delta]: ./readme_images/delta.png
[delta-calc]: ./readme_images/delta-calc.png
[WC]: ./readme_images/WC.png
[WC^1]: ./readme_images/WC^1.png
[theta_3-zoom]: ./readme_images/theta_3-zoom.png
[theta_3-calc]: ./readme_images/theta_3-calc.png
[epsilon]: ./readme_images/epsilon.png
[epsilon-calc]: ./readme_images/epsilon-calc.png
[T]: ./readme_images/T.png
[R]: ./readme_images/R.png
[R-calc]: ./readme_images/R-calc.png
[R_0_6]: ./readme_images/R_0_6.png
[R_3_6]: ./readme_images/R_3_6.png
[R_rpy-calc]: ./readme_images/R_rpy-calc.png
[R_3_6-calc-LHS-1]: ./readme_images/R_3_6-calc-LHS-1.png
[R_3_6-calc-LHS-2]: ./readme_images/R_3_6-calc-LHS-2.png
[y]: ./readme_images/y.png
[P_small]: ./readme_images/p.png

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

Following are DH parameters used specifically in this project:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0

**Homogenous transforms** are then combined together. Parameters of each transformation are set from **DH parameters**.Each transformation matrix looks like this:

![^{i-1}_iT=\begin{bmatrix}cos(\theta_i) &  - sin(\theta_i) & 0 & a \\ sin(\theta_i)cos(\alpha_{i-1}) & cos(\theta_i)cos(\alpha_{i-1}) &  - sin(\alpha_{i-1}) &  - d  *  sin(\alpha_{i-1}) \\ sin(\theta_i)sin(\alpha_{i-1}) & cos(\theta_i)sin(\alpha_{i-1}) & cos(\alpha_{i-1}) & d  *  cos(\alpha_{i-1}) \\ 0 & 0 & 0 & 1 \end{bmatrix}][transform-single]

Simplified as ![^{0}_1T][transform-simple] which means *tranformation from reference frame A to reference frame B*.

It is important to note (for later when we calculate ![q4][theta4] to ![q6][theta6] in this project) that this transformation matrix is consisted of rotational and translational matrices:

![transform-composition1][transform-composition1]

![^{A}_BR][A_B_R] denotes the rotational matrix from frame A to frame B. In other words, then, ![R][R] block from [T][T] matrix above looks as follows:

![^{i-1}_iR=\begin{bmatrix}cos(\theta_i) &  - sin(\theta_i) & 0 \\ sin(\theta_i)cos(\alpha_{i-1}) & cos(\theta_i)cos(\alpha_{i-1}) &  - sin(\alpha_{i-1}) \\ sin(\theta_i)sin(\alpha_{i-1}) & cos(\theta_i)sin(\alpha_{i-1}) & cos(\alpha_{i-1})  \end{bmatrix}][rotation-single]

The links go from 0 to 6 and then followed by EE, that is why in the DH parameters above we have 7 rows. To combine transformations, calculate the dot products of all single transformations:

![^{0}_{EE}T=^{0}_{1}T * ^{1}_{2}T * ^{2}_{3}T * ^{3}_{4}T * ^{4}_{5}T * ^{5}_{6}T * ^{6}_{EE}T][transform-comb]



