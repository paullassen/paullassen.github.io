---
layout: default
title: Master's Thesis
parent: Projects
nav_order: 0
---
# **Modelling and Control of an Aerial Manipulator**
On this page I'll present a summary of the work I did for my master's thesis. Some of the work in the thesis was spun off into a paper, submitted to and accepted at the *AIRPHARO 2021 Workshop on Aerial Robotic Systems Physically Interacting with the Environment*.

The full thesis can be found [here](/assets/MSC_Thesis.pdf). 

## Abstract
Aerial robots have historically been used as flying eyes in the skies. Mounted with sensory payloads, they have been content to use their agility and range to see, smell, and listen to the world around them. More recently, research has begun to tackle the challenges of physical interaction. Unlocking this capability would enable UAVs to take on a much broader range of applications, including tasks which currently pose a risk of injury to the people performing them.

In this post, I'll address the issue of performing physical interaction with the environment by a multi-rotor UAV implementing a basic cascaded position-attitude controller, typical of most of consumer-grade multi-rotor systems. More specifically, I identify mathematically the boundaries where the system can safely be used to perform physical interaction with the environment. These boundaries are then verified both in simulation and by experiment.

## System Modelling
It is convenient to start by laying out the assumptions that I make while building the mathematical model of the drone platform in flight and in contact with the environment. 

### Assumptions
1. The manipulator is of neglible mass and rigidly attached to the center of mass of the UAV.
2. The UAV is assumed to be moving slowly enough, both in free-flight and in contact, for aerodynamic forces such as air resistance to be neglible.
3. The contact surface is a plane parallel to the \\( y,z \\) plane of the intetial world frame.
4. Contact between the UAV and the contact surface occurs 'head on', where the UAV is pitching, but roll, \\( \theta \\), and yaw, \\( \psi \\), are zero.

### Nomenclature 

<table>
<colgroup>
<col width="10%" />
<col width="90%" />
</colgroup>
<thead>
<tr class="header">
<th>Symbol</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td markdown="span"> $$ W $$ </td>
<td markdown="span"> The inertial world frame, fixed at some point in space</td>
</tr>
<tr>
<td markdown="span"> $$ B $$ </td>
<td markdown="span"> The body frame, fixed to the center of mass of the UAV</td>
</tr>
<tr>
<td markdown="span"> $$ E $$ </td>
<td markdown="span"> The end-effector frame, fixed to the tip of the manipulator</td>
</tr>
<tr>
<td markdown="span"> $$ {\xi}^\circ_\star $$ </td>
<td markdown="span"> The position of the origin of frame \\\\( \star \\\\) w.r.t. frame \\\\( \circ \\\\).  
\\\\( \xi^\circ_\star = \begin{bmatrix} x^\circ_\star & y^\circ_\star  & z^\circ_\star  \end{bmatrix}^T \\\\) </td>
</tr>
<tr>
<td markdown="span"> $$ \dot{\xi}^\circ_\star $$ </td>
<td markdown="span"> The velocity of the origin of frame \\\\( \star \\\\) w.r.t. frame \\\\( \circ \\\\).  
\\\\( \dot{\xi}^\circ_\star = \begin{bmatrix} \dot{x}^\circ_\star & \dot{y}^\circ_\star  & \dot{z}^\circ_\star  \end{bmatrix}^T \\\\) </td>
</tr>
<tr>
<td markdown="span"> $$ \ddot{\xi}^\circ_\star $$ </td>
<td markdown="span"> The acceleration of the origin of frame \\\\( \star \\\\) w.r.t. frame \\\\( \circ \\\\).  
\\\\( \ddot{\xi}^\circ_\star = \begin{bmatrix} \ddot{x}^\circ_\star & \ddot{y}^\circ_\star  & \ddot{z}^\circ_\star  \end{bmatrix}^T \\\\) </td>
</tr>
<tr>
<td markdown="span"> $$ R^\circ_\star $$ </td>
<td markdown="span"> The rotation matrix representing the orientation of frame $$\star$$ with respect to frame $$\circ$$.  </td>
</tr>
<tr>
<td markdown="span"> $$ \eta $$ </td>
<td markdown="span"> The euler angle parameterization of the orientation of the body frame with respect to the inertial frame.  
\\\\( \eta = \begin{bmatrix} \phi & \theta & \psi\end{bmatrix}^T \\\\) </td>
</tr>
<tr>
<td markdown="span"> $$ \dot{\eta} $$ </td>
<td markdown="span">  Time derivative of the euler angles.  
\\\\( \dot{\eta} = \begin{bmatrix} \dot{\phi} & \dot{\theta} & \dot{\psi} \end{bmatrix}^T \\\\) 
</td>
</tr>
<tr>
<td markdown="span"> $$ \nu $$ </td>
<td markdown="span">  Instantaneous angular velocity of the UAV with respect to the body frame.  
\\\\( \nu = \begin{bmatrix} p & q & r \end{bmatrix}^T \\\\)</td>
</tr>
<tr>
<td markdown="span"> $$ \dot{\nu} $$ </td>
<td markdown="span"> Instantaneous angular acceleration of the UAV with respect to the body frame.  
\\\\( \dot{\nu} = \begin{bmatrix} \dot{p} & \dot{q} & \dot{r} \end{bmatrix}^T \\\\)</td>
</tr>
<tr>
<td markdown="span"> $$ m $$ </td>
<td markdown="span"> The mass of the UAV </td>
</tr>
<tr>
<td markdown="span"> $$ g $$ </td>
<td markdown="span"> The acceleration due to gravity</td>
</tr>
<tr>
<td markdown="span"> $$ \textbf{I} $$ </td>
<td markdown="span"> The moment of inertia of the UAV</td>
</tr>
<tr>
<td markdown="span"> $$ f^\circ $$ </td>
<td markdown="span"> Force described in frame $$ \circ $$</td>
</tr>
<tr>
<td markdown="span"> $$ \tau^\circ $$ </td>
<td markdown="span"> Torque described in frame $$ \circ $$</td>
</tr>
<tr>
<td markdown="span"> $$ S_\alpha, C_\alpha, T_\alpha $$ </td>
<td markdown="span"> Shorthand for $$ \sin(\alpha), \cos(\alpha), \tan(\alpha) $$</td>
</tr>
</tbody>
</table>


### General Model
For this work I considered a coaxial hexarotor platform with a rigidly attached manipulator. A diagram of the base platform without the manipulator is shown in fig 1. The manipulator is a rod of length \\( L_m \\) attached rigidly to the center of mass of the UAV, extending straight out between rotors 1 and 6, in the direction indicated by the red arrow, along the \\( x \\) axis.

![](/assets/Thesis_page_4.png)
*Fig. 1.  Diagram of a coaxial hexarotor. The body frame  \\( B \\)  is attached to the center of mass of the UAV. The direction of the \\( x \\) and \\( y \\) axes are shown, the \\( z \\) projects out towards the reader. The rotors are arranged in counter-rotating pairs.*


### Contact Model

## Controller

### Cascaded Controllers

### Controllers in Contact

## Verification

### Simulation

### Experiment

## Conclusion


