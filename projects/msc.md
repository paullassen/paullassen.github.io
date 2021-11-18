---
layout: default
title: Thesis
parent: Projects
nav_order: 21
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
<details>
<summary> (<i>click to expand</i>) </summary>

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
<td markdown="span"> The position of the origin of frame \\( \star \\) w.r.t. frame \\( \circ \\).  
\\( \xi^\circ_\star = \begin{bmatrix} x^\circ_\star & y^\circ_\star  & z^\circ_\star  \end{bmatrix}^T \\) </td>
</tr>
<tr>
<td markdown="span"> $$ \dot{\xi}^\circ_\star $$ </td>
<td markdown="span"> The velocity of the origin of frame \\( \star \\) w.r.t. frame \\( \circ \\).  
\\( \dot{\xi}^\circ_\star = \begin{bmatrix} \dot{x}^\circ_\star & \dot{y}^\circ_\star  & \dot{z}^\circ_\star  \end{bmatrix}^T \\) </td>
</tr>
<tr>
<td markdown="span"> $$ \ddot{\xi}^\circ_\star $$ </td>
<td markdown="span"> The acceleration of the origin of frame \\( \star \\) w.r.t. frame \\( \circ \\).  
\\( \ddot{\xi}^\circ_\star = \begin{bmatrix} \ddot{x}^\circ_\star & \ddot{y}^\circ_\star  & \ddot{z}^\circ_\star  \end{bmatrix}^T \\) </td>
</tr>
<tr>
<td markdown="span"> $$ R^\circ_\star $$ </td>
<td markdown="span"> The [rotation matrix](https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix) representing the orientation of frame $$\star$$ with respect to frame $$\circ$$.  </td>
</tr>
<tr>
<td markdown="span"> $$ \eta $$ </td>
<td markdown="span"> The euler angle parameterization of the orientation of the body frame with respect to the inertial frame.  
\\( \eta = \begin{bmatrix} \phi & \theta & \psi\end{bmatrix}^T \\) </td>
</tr>
<tr>
<td markdown="span"> $$ \dot{\eta} $$ </td>
<td markdown="span">  Time derivative of the euler angles.  
\\( \dot{\eta} = \begin{bmatrix} \dot{\phi} & \dot{\theta} & \dot{\psi} \end{bmatrix}^T \\) 
</td>
</tr>
<tr>
<td markdown="span"> $$ \nu $$ </td>
<td markdown="span">  Instantaneous angular velocity of the UAV with respect to the body frame.  
\\( \nu = \begin{bmatrix} p & q & r \end{bmatrix}^T \\)</td>
</tr>
<tr>
<td markdown="span"> $$ \dot{\nu} $$ </td>
<td markdown="span"> Instantaneous angular acceleration of the UAV with respect to the body frame.  
\\( \dot{\nu} = \begin{bmatrix} \dot{p} & \dot{q} & \dot{r} \end{bmatrix}^T \\)</td>
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
</details>

### General Model
For this work I considered a coaxial hexarotor platform with a rigidly attached manipulator. A diagram of the base platform without the manipulator is shown in fig 1. The manipulator is a rod of length \\( L_m \\) attached rigidly to the center of mass of the UAV, extending straight out between rotors 1 and 6, in the direction indicated by the red arrow, along the \\( x \\) axis.

![](/assets/Thesis_page_4.png)
*Fig. 1.  Diagram of a coaxial hexarotor. The body frame  \\( B \\)  is attached to the center of mass of the UAV. The direction of the \\( x \\) and \\( y \\) axes are shown, the \\( z \\) projects out towards the reader. The rotors are arranged in counter-rotating pairs.*

I define three frames of reference \\( W \\) , \\( B \\) , and \\( E \\), representing the inertial world frame, the body-fixed frame and the end-effector-fixed frame. It is useful to note that due to assumption 1, the axes of \\( E \\) are always parallel to the axes of \\( B \\).

<details>
<summary> (<i> Click to see details </i>) More details can be found on pages 6-9</summary>

The six rotors are arranged as shown in fig. 1. Each rotor is centered at a point  
$$ \mathbf{r}^B_i = L_r \begin{bmatrix} \cos(60i - 30) \\ -\sin(60i - 30) \\ 0 \end{bmatrix} $$

where \( L_r \) is the distance from the center of mass of the UAV to the center of each rotor. The 
\( i^{th} \) rotor generates a force \( \mathbf{f}^B_i \) and a torque \( \tau^B_i \) given by  
$$ \mathbf{f}^B_i = k \begin{bmatrix} 0 \\ 0 \\ \omega^2_i \end{bmatrix} \tag{1} $$

$$ \tau^B_i = S(\mathbf{r}^B_i) \mathbf{f}^B_i + b \begin{bmatrix} 0 \\ 0 \\ \omega^2_i \end{bmatrix} + I_M \begin{bmatrix} 0 \\ 0 \\ \dot{\omega_i} \end{bmatrix} \tag{2} $$
where \( \omega_i \) is the angular velocity of the \( i^{th} \) rotor, \( k \) is the lift constant, \( b \) is the drag constant, \( I_M \) is the momnet of inertia of the rotor, and \( S(\cdot) \) maps a vector to its [skew symmetric matrix](https://en.wikipedia.org/wiki/Skew-symmetric_matrix).

The derivative motor term \( I_M \dot{\omega_i} \) is omitted from the rest of the derivation due to it's small value.

The total thrust force \( \mathbf{f}^B_t \) generated by the rotors is found by simply summing the various forces generated by each rotor. Due to the layout of the rotors on a coaxial multirotor, the force generated by each rotor occurs along the \( z \) axis of the body frame. The total thrust force then only has one non-zero component ( in \( B \) ) denoted \( T \).
$$ \mathbf{f}^B_t = \sum^6_{i = 1} \mathbf{f}^B_i = \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} \tag{3} $$

The total torque is calculated in the same way.  
$$ \tau^B_t = \sum^6_{i = 1} \tau^B_i = \begin{bmatrix} \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} \tag{4} $$
 
The total wrench generated by the UAV is a vector with 6 elements, 3 elements of force and 3 elements of torque. Of these 6 elements, 4 of them are non-zero, the thrust force along the \( z \) axis and torques around each of the axes of \( B \). The reduced wrench vector \( \mathbf{w}^B_r \) contains only these active elements.
$$ \mathbf{w}^B_{total} = \begin{bmatrix} \mathbf{f}^B_t \\ \tau^B_t \end{bmatrix} = \begin{bmatrix}0 \\ 0 \\ T \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ \mathbf{w}^B_r \end{bmatrix} \tag{5} $$

The linear dynamics of the UAV in the inertial world frame \( W \) are given by  
$$ m \ddot{\xi}^W_B = \mathbf{f}^W_g + \mathbf{R}^W_B \mathbf{f}^B_t \tag{6} $$
where  
$$ \mathbf{f}^W_g = \begin{bmatrix} 0 \\ 0 \\ -mg \end{bmatrix} \tag{7}$$  
is the force of gravity and \( \mathbf{R}^W_B \) is the rotation matrix, presenting the orientation of \( B \) with respect to \( W \).

The rotation matrix \( \mathbf{R}^W_B\) is parameterized by \( \eta = \begin{bmatrix} \phi & \theta & \psi \end{bmatrix}^T \), using the [yaw-pitch-roll convention](https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles).  

$$ \mathbf{R}^W_B = R_z (\psi) R_y (\theta) R_x (\phi) = \begin{bmatrix}C_\psi C_\theta  & C_\psi S_\phi S_\theta - C_\phi S_\psi  & S_\phi S_\psi + C_\phi C_\psi S_\theta \\ C_\theta S_\psi  & C_\phi C_\psi + S_\phi S_\psi S_\theta  & C_\phi S_\psi S_\theta - C_\psi S_\phi \\ -S_\theta  & C_\theta S_\phi  & C_\phi C_\theta\end{bmatrix} \tag{8} $$

where \( R_z (\psi) \) represents a rotation of \( \psi \) around the \( z \) axis, followed by \( R_y (\theta) \) around the new \( y \) axis and then \( R_x (\phi) \) around the final \( x \) axis. The inverse operation, that is the rotation matrix from the inertial frame to the body frame, is given by

$$ R^B_W = \left(R^W_B\right)^{-1} = \left(R^W_B\right)^T $$

by the general properties of rotation matrices.

Putting equations 3, 6, 7, and 8 together reveal the linear dynamics of the UAV

$$ m \ddot{\xi}^W_B = \begin{bmatrix}  T (S_\phi S_\psi + C_\phi C_\psi S_\theta) \\ -T (C_\psi S_\phi - C_\phi S_\psi S_\theta) \\ T C_\phi C_\theta - m g\end{bmatrix} \tag{9}$$

It's worth noting that, for the UAV to remain airborne, it needs to maintain zero acceleration along the \( z \) axis of the inertial frame \( W \). From equation 9,   

$$ m \ddot{z}^W_B = T C_\theta C_\phi - mg = 0 $$  

Solving for \( T \) to find the thrust required to hover, reveals  

$$ T_{hover} = \frac{mg}{C_\theta C_\phi} \tag{10} $$

The rotational dynamics of the UAV in \( B \) are given by 

$$ \mathbf{I}\dot{\nu} + \nu \times (\mathbf{I}\nu) = \tau^B_t \tag{11} $$
Solving equation 11 for the angular aceleration \( \dot{\nu} \), 

$$ \dot{\nu} = \mathbf{I}^{-1}\left(\tau^B_t - \nu \times \left(\mathbf{I}\nu\right)\right) $$

The transformation from the angular velocity of the UAV with respect to \( W \) to the angular velocity of the UAV with respect to \( B \) can be expressed as a matrix \( \mathbf{W}_\eta \). 

$$ \nu = \mathbf{W}_\eta \dot{\eta} $$ 

$$ \dot{\eta} = \mathbf{W}^{-1}_\eta \nu $$

where 

$$ \mathbf{W}_\eta = \begin{bmatrix} 1 & 0 & -S_\theta \\ 0 & C_\theta & C_\theta S_\phi \\ 0 & -S_\phi & C_\theta C_\phi \end{bmatrix} $$

</details>

The Newton-Euler equations of motion describing the UAV are:

$$ m \ddot{\xi}^W_B = \mathbf{f}^W_g + \mathbf{R}^W_B \mathbf{f}^B_t $$

$$ \dot{\nu} = \mathbf{I}^{-1}\left(\tau^B_t - \nu \times \left(\mathbf{I}\nu\right)\right) $$


### Motor Mixer

At this point both the linear and rotational dynamics of the UAV have been derived and we are just about ready to move onto the contact model. Before we do so, I'm going to introduce the Motor Mixer Matrix. The Motor Mixer takes a set of desired forces and torques \\( \mathbf{w}^B_r \\) and outputs the (squared) motor speeds \\( \Omega \\) required to achieve them.

<details>
<summary> (<i> Click to see details </i>) More details can be found in 3.3.1 on pages 14-17</summary>

It is evident from equations 2 and 3 that the force and torque are functions of the squared rotor speeds. This fact carries through to equation 6. It is convenient for future derivations to take a look at the reduced wrench vector \( \mathbf{w}^B_r \) w.r.t. the squared motor speeds.

First we define the vector \( \Omega \) to be the vector of squared motor speeds.

$$ \Omega = \begin{bmatrix} \omega^2_1 \\ \omega^2_2 \\ \cdots \\ \omega^2_6 \end{bmatrix} $$

It is now possible to descibe the reduced wrench vector as a function of \( \Omega \).

$$ \mathbf{w}^B_r = \mathbf{J} \Omega $$

where \( \mathbf{J} \in \mathbb{R}^{4 \times 6} \) is the Jacobian of \( \mathbf{w}^B_r \) w.r.t. \( \Omega \). The pseudoinverse of the Jacobian

$$ M = J^+ $$ 

is called the motor mixer matrix and maps some reduced wrench to the (squared) rotor speeds required to achieve it.

</details>

$$ \Omega = M\mathbf{w}^B_r $$

### Contact Model

For the contact model, I consider the UAV with its end-effector in contact with the wall. The UAV is considered to be in static equilibrium, that is, not moving or accelerating, linearlly or rotationally. The goal of the contact model is to identify the limits of the UAVs ability to maintain contact and a static equilibrium, a state I refer to as **static contact**.

I found that the UAVs ability to maintain static contact is defined by two limits. The first is a requirement that the friction force experienced by the end-effector is sufficient to prevent the end-effector from slipping. The second is the limit of the UAVs ability to counteract the reaction torque from the contact forces experienced by the end-effector. 

<details>
<summary> (<i> Click to see details </i>) More details can be found in 4.1 on pages 40-9</summary>

As the UAV applies a force on the wall, a countervailing force is applied on the end-effector of the UAV. This force is made up of two parts, a normal force and a friction force. The normal force, on the other hand, works to prevent the end-effector from penetrating the contact surface. The friction force occurs parallel to the contact surface and works to prevent the end-effector from slipping along the wall. The magnitude of the force of friction is proportional to the normal force.

From assumption 3 it follows that the contact force experienced by the UAV at the end-effector is,

$$ \mathbf{f}^W_c = \begin{bmatrix} f_n \\ f_{f,y} \\ f_{f,z} \end{bmatrix} $$

During static contact the following must be true,

$$ \lvert f_f \rvert  \leq \mu \lvert f_n \rvert $$

When the UAV is in static contact with the wall, 

$$ m \ddot{\xi}^W_B = \mathbf{f}^W_g + \mathbf{R}^W_B \mathbf{f}^B_t + \mathbf{f}^W_c = 0$$
$$ \mathbf{f}^W_c =-\mathbf{f}^W_g - \mathbf{R}^W_B \mathbf{f}^B_t = \left\lbrack\begin{array}{c} -T(S_\phi S_\psi + C_\phi C_\psi S_\theta) \\ -T(C_\phi S_\psi S_\theta - C_\psi S_\phi) \\ -T C_\phi C_\theta+m g\end{array}\right\rbrack $$

where T is the thrust force generated by the UAV. 

Applying assumption 4 and the friction inequality reveal the limits imposed by friction of the UAVs ability to maintain static contact.

$$ \lvert T C_\theta - m g \rvert \leq \mu \lvert T S_\theta \rvert $$

<p>
	<img src="/assets/friction_bound_0_7.png">
</P>
*Fig 2.* ***Friction Bound*** *The yellow region represents the region of thrust force and pitch angle where the friction force is large enough to prevent the end-effector froms slipping along the wall. The yellow region is plotted with \\(\mu=0.7\\).*

Friction does not provide the only limit to the UAVs ability to maintain static contact. The UAV must also be able to counteract the reaction torque from the contact forces applied at the end effector. The reaction torque is given by,

$$ \tau^B_c = S\left( \xi^B_E \right) \mathbf{R}^B_W \mathbf{f}^W_c = L_m \left\lbrack\begin{array}\\ 0 \\ T - m g C_\theta \\ 0 \end{array} \right\rbrack $$ 

To assess the limits of the UAVs ability to counteract the reaction torque, we set the desired wrench and pass it through the motor mixer.

$$ \Omega_c = M \mathbf{w}^B_r = \mathbf{M} \begin{bmatrix} T \\ -\tau^B_c \end{bmatrix} = \frac{1}{6k}\left\lbrack\begin{array}{l}
T + \sqrt{3} s_{mr} (T -  m g C_\theta) \\
T \\
T - \sqrt{3} s_{mr} (T -  m g C_\theta) \\
T - \sqrt{3} s_{mr} (T -  m g C_\theta) \\
T \\
T + \sqrt{3} s_{mr} (T -  m g C_\theta) \\
\end{array}\right\rbrack $$

where \\(s_{mr} = L_m/L_r \\) is the ratio of manipulator length to rotor length.

The standard layout for coplanar multirotor craft uses brushless dc motors configured to run in only one direction. This fact requires the rotor speeds (and their squares) to be non-negative. Of particular note here, the 3rd and 4th rotor speeds.

$$ T - \sqrt{3} s_{mr} (T - m g C_\theta) \geq 0 $$

![](/assets/torque_bound.png)
*Fig 3.* ***Torque Bound*** *The blue region represents the region of thrust force and pitch angle in which the UAV is capable of counteracting the reaction torque on the UAV from contact. Outside of this region, the UAV is unable to maintain its orientation and will begin to pitch into the wall. The blue region is plotted with \\(s_{mr} = 2\\), the yellow and cyan dashed lines represent \\(s_{mr}=1.5\\) and \\(s_{mr}=3.4\\) respectively.*

</details>

The **operating region** of the UAV is the region of thrust force and pitch angle in where both the friction bound and the torque bound are satisfied.

![](/assets/theta_lim_contour_0_7.png)
*Fig 4.* ***Operating Region*** *Shown in green, the region of Thrust Force and Pitch in which the UAV is physically capable of maintaining* ***static contact*** *with the wall.*


## Controllers in Contact
To understand how our control scheme affects the UAVs operating region, I'll start by defining the controllers. The position control scheme used for this thesis was a cascaded PID control scheme, with an outer PD position control loop feeding attitude references into an inner PD attitude control loop, which, along with a PD altitude controller passed a reduced wrench to the Motor Mixer.

![](/assets/ctrl_schematic.png)
*Fig 5.* ***Cascaded Position Controller***

The altitude controller, attitude controllers, and position controllers were implemented as follows:

$$ T_{ref} = \frac{1}{C_\phi C_\theta} (k_{p,z} e_{p,z} + k_{d,z} e_{d,z} + m g) $$
 
$$ \tau_{ref} = \mathbf{K}_{p,\eta} \mathbf{e}_{p,\eta} + \mathbf{K}_{d,\eta} \mathbf{e}_{d,\eta} $$
 
$$ \phi_{ref} = - (k_{p,y} e'_{p,y} + k_{d,y} e'_{d,y} )$$
 
$$ \theta_{ref} = k_{p,x} e'_{p,x} + k_{d,x} e'_{d,x} $$
  

![](/assets/real_reg_0_7.png)
*Fig 3.* ***Realisable Region*** *The region of Thrust Force and Pitch in which the PID attitude controller of the UAV is capable of maintaining* ***static contact*** *with the wall.*

## Verification

### Simulation

### Experiment

## Conclusion


