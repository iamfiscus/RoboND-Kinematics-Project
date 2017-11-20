## Project: Kinematics Pick & Place

### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Steps to complete the project:**

1. Set up your ROS Workspace.
2. Download or clone the
   [project repository](https://github.com/udacity/RoboND-Kinematics-Project)
   into the **_src_** directory of your ROS Workspace.
3. Experiment with the forward_kinematics environment and get familiar with the
   robot.
4. Launch in
   [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the
   [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.

[//]: # "Image References"
[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[diagram]: ./misc_images/diagram.png
[angle]: ./misc_images/angle.png
[grasp]: ./misc_images/grasp.png
[move-path]: ./misc_images/move-path.png
[dh-transform-matrix]: ./misc_images/dh-transform-matrix.png
[thetas456]: ./misc_images/theta456.gif
[matrices]: ./misc_images/matrices.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it!

### Kinematic Analysis

#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

First I started by looking at the kr20.urdf.xacro as a reference to get the XYZ,
roll, pitch, and yaw values for the joints. This table mimics is a visual
representation of the XML.

| Joint                      | Parent       | Child                     | x     | y   | z      | roll | pitch | yaw |
| -------------------------- | ------------ | ------------------------- | ----- | --- | ------ | ---- | ----- | --- |
| joint_1                    | base_link    | link_1                    | 0     | 0   | 0.33   | 0    | 0     | 0   |
| joint_2                    | link_1       | link_2                    | 0.35  | 0   | 0.42   | 0    | 0     | 0   |
| joint_3                    | link_2       | link_3                    | 0     | 0   | 1.25   | 0    | 0     | 0   |
| joint_4                    | link_3       | link_4                    | 0.96  | 0   | -0.054 | 0    | 0     | 0   |
| joint_5                    | link_4       | link_5                    | 0.54  | 0   | 0      | 0    | 0     | 0   |
| joint_6                    | link_5       | link_6                    | 0.193 | 0   | 0      | 0    | 0     | 0   |
| gripper_joint              | link_6       | gripper_link              | 0.11  | 0   | 0      | 0    | 0     | 0   |
| right_gripper_finger_joint | gripper_link | right_gripper_finger_link | 0.15  | 0   | 0      | 0    | 0     | 0   |
| left_gripper_finger_joint  | gripper_link | left_gripper_finger_link  | 0.15  | 0   | 0      | 0    | 0     | 0   |

Then follow this model to derive the DH parameters based on the lesson.

![alt text][diagram]

| i   | a(i-1) | a(i-1) | d(i) | θ(i)    |
| --- | ------ | ------ | ---- | ------- |
| 1   | 0      | 0      | d1   | θ1      |
| 2   | -90    | a1     | 0    | θ2 - 90 |
| 3   | 0      | a2     | 0    | θ3      |
| 4   | -90    | a3     | d4   | θ4      |
| 5   | 90     | 0      | 0    | θ5      |
| 6   | -90    | 0      | 0    | θ6      |
| G   | 0      | 0      | dG   | 0       |

| Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i) |
| ----- | ---------- | ------ | ------ | -------- |
| 0->1  | 0          | 0      | 0.75   | 01       |
| 1->2  | -90        | 0.35   | 0      | 02 - 90  |
| 2->3  | 0          | 1.25   | 0      | 03       |
| 3->4  | -90        | -0.054 | 0      | 04       |
| 4->5  | 90         | 0      | 1.5    | 05       |
| 5->6  | -90        | 0      | 0      | 06       |
| 6->EE | 0          | 0      | 0.303  | 0        |

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

This is the code that gives us our DH table print out below.

```python
DH_Table = { alpha0:      0, a0:      0, d1:  0.75, q1:          q1,
               alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2. + q2,
               alpha2:      0, a2:   1.25, d3:     0, q3:          q3,
               alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:          q4,
               alpha4: -pi/2., a4:      0, d5:     0, q5:          q5,
               alpha5: -pi/2., a5:      0, d6:     0, q6:          q6,
               alpha6:      0, a6:      0, d7: 0.303, q7:           0}
```

In order to create individual transformation matrices we are first going to
create a reusable function based on the following equation:
![alt text][dh-transform-matrix]

> Referenced from Udacity Robotics Nanodegree

```python
def TF_Matrix(alpha, a, d, q):
        TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                     [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                     [                 0,                 0,           0,             1]])
        return TF
```

Here are the equations for the matrices:

![equations][matrices]

<!--
$^0_1 T =
 \begin{pmatrix}
  c\theta_1 & -s\theta_1 & 0 & 0 \\
  s\theta_1 & c\theta_1  & 0 & 0 \\
  0 & 0 & 1 & 0.75 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$

$^1_2 T =
 \begin{pmatrix}
  s\theta_2 & c\theta_2 & 0 & 0.35 \\
  0 & 0 & 1 & 0 \\
  c\theta_2 & -s\theta_2  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$

$^2_3 T =
 \begin{pmatrix}
  c\theta_3 & -s\theta_3 & 0 & 1.25 \\
  s\theta_3 & c\theta_3  & 0 & 0 \\
  0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$

$^3_4 T =
 \begin{pmatrix}
  c\theta_4 & -s\theta_4 & 0 & -0.054 \\
  0 & 0 & 1 & 1.5 \\
  -s\theta_4 & -c\theta_4  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$

$^4_5 T =
 \begin{pmatrix}
  c\theta_5 & -s\theta_5 & 0 & 0 \\
  0 & 0 & -1 & 0 \\
  s\theta_5 & c\theta_5  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$

$^5_6 T =
 \begin{pmatrix}
  c\theta_6 & -s\theta_6 & 0 & 0 \\
  0 & 0 & 1 & 0 \\
  -s\theta_6 & -c\theta_6  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$

$^6_{gripper} T =
 \begin{pmatrix}
  1 & 0& 0 & 0 \\
  0 & 1 & 0 & 0 \\
  0 & 0  & 1 & 0.303 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$

 https://www.madoko.net/editor.html
 -->

Then we can create our transformation matrices and links in a more readable way.

```python
T0_1  = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2  = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3  = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4  = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5  = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6  = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
Wrist center(WC) is created with the following function using joint 5.

```python
WC = EE - (0.303) * ROT_EE[:,2]
```

Joints 1-3 are used in order to position the WC correctly which is inverse position kinematics. They are called Theta 1-3 in the code.
Theta 1 is calculated using WC array.

```python
theta1 = atan2(WC[1], WC[0]) # Equation = atan2(WCy, WCx)
```

We get Theta2 with SSS triangle constructed using joint 2, joint 3, and WC.

![Angles][angle]

```python
# Equation = pi /2 - angle_a - atan2(WCz - 0.75, sqrt(WCx * WCx + WCy * WCy) - 0.35)
theta2 = pi /2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
```

We get Theta 3 with the following:

```python
theta3 = pi / 2 - (angle_b + 0.036)
```

The last joints 4-6 are used in order to orient the end effector which is inverse orientation kinematics. They are called Theta 4-6 in the code.
In Thetas 4-6 you can use the following equation: ![Thetas 4, 5, 6][thetas456]

<!-- \begin{bmatrix}
 & r00, r01, r02 & \\
 & r10, r11, r12 & \\
 & r20, r21, r22 &
\end{bmatrix}= \begin{bmatrix}
 & -sin(\Theta 4)sin(\Theta 6)+cos(\Theta 4)cos(\Theta 5)cos(\Theta 6), -sin(\Theta4)cos(\Theta6)-sin(\Theta 6)cos(\Theta4)cos(\Theta5), -sin(\Theta5)cos(\Theta 4) & \\
 & sin(\Theta5)cos(\Theta6), -sin(\Theta5)sin(\Theta6), cos(\Theta5) & \\
 & -cos(\Theta4)sin(\Theta6)-sin(\Theta4)cos(\Theta5)cos(\Theta 6), -cos(\Theta4)cos(\Theta6)+sin(\Theta4)cos(\Theta5)*sin(\Theta6), sin(\Theta4)sin(\Theta5) &
\end{bmatrix}

Editor:
https://www.codecogs.com/latex/eqneditor.php

 -->

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

This was a very hard project for me, so I ended up using the walkthrough for
help.

However the first step was defining the symbols and then calculating the
transform matrix between the different joints. I made a reusable function which
takes the parameters alpha, a, d, and q. `TF_Matrix(alpha, a, d, q)`

Next was to do the inverse position kinematics and inverse orientation
kinematics which I had to calculate rotation matrix from the given roll, pitch
and yaw. I then had to calculate the rotation matrix ROT_EE which I used the
equation to find WC.

#### Result

Here is an example of the robot grasping the cylinder: ![Grasping][grasp]

Here is an example of the robot trajectory path: ![Trajectory Path][move-path]
