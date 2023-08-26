# Course 1: Foundations of Robot Motion

## Configuration Space

Robot configuration: a specification of the positions of all points of the robot.

## Degree of Freedom

The number of degrees of freedom (dof) of a robot is the smallest number of real-valued coordinates needed to represent its configuration.
<br>
degrees of freedom = (sum of freedoms of the bodies) − (number of independent constraints).
<br>
A 2D planner body can have maximum 3 DOF - 2 translation in the plane itself and one rotation about the axis coming out the plane.
<br>
A 3D rigid body can have maximum 6 DOF - 3 in Rotational and 3 in translation about the unit axis x,y,z.


## Grubler's Formula

Consider a mechanism consisting of N links, where ground
is also regarded as a link. Let J be the number of joints, m be the number of
degrees of freedom of a rigid body (m = 3 for planar mechanisms and m = 6 for
spatial mechanisms), f<sub>i</sub> be the number of freedoms provided by joint i, and ci be
the number of constraints provided by joint i, where fi + ci = m for all i. Then Degree of Freedom (DOF) is given by, 


$$
DOF = m (N-1-J) + \sum_{i=1}^{J} f_i
$$

## C-Space Topology

Topology can be considered as the shape or the orientation of the surface.
<br>
Two spaces are topologically equivalent if one can be continuously deformed into the other without cutting or gluing.
<br>
Topology is independent of how we represent the space.

## C-space representation

An implicit representation views the n-dimensional space as embedded in a Euclidean space of more than n dimensions.

<p align="center">
	<img src="Images/fourbarlinkage.png" width="586" height="328"/>
</p>

The DOF of above 4-Bar linkage can we calculated using Grubler's formula which is 1.<br>
1 DOF implies that the orientation of this 4-Bar linkage mechanism can be represented using single parameter/variable, which will be hard to derive and may contain singularities.
<br>
PS: A singularity is a particular point in a robot's workspace that causes the robot to lose one or more degrees of freedom (DoF)
<br>
Instead we can the orientation of the mechanism using four parameters that is with 4-DOF subject to to 3 constrains leading to 1-DOF.
<br>
This can be alternatively stated as 1D space embed in 4D space of θ<sub>1</sub>, θ<sub>2</sub>, θ<sub>3</sub> and θ<sub>4</sub>.


$$
L_1 cos(\theta_1) + L_2 cos(\theta_1+\theta_2) + ... + L_4 cos(\theta_1 + ...+ \theta_1) = 0
$$


$$
L_1 sin(\theta_1) + L_2 sin(\theta_1+\theta_2) + ... + L_4 sin(\theta_1 + ...+ \theta_1) = 0
$$

$$
\theta_1 + \theta_1 +\theta_1 + \theta_1 - 2\pi= 0
$$

<br>
Holonomic constrains reduce the C-space and thus the DOF of the robot.
<br>
Non-holonomic constraints do not limit the C-space dimension but limit the motion of the robot (e.g. translation motion of a car).

<br>

**Velocities are not the time derivatives of coordinates.**

## Task Space and Workspace

The task space is a space in which the robot’s task can be naturally ex-
pressed. We only need to know about the task and not the robot to define the task space.

<br>
The workspace is a specification of the configurations that the -effector of the robot can reach. The workspace is independent of task the robot has to do and depends on the range of motion of the robot.


## Rigid Body Motion

The configuration and position of a rigid body can be represented using a 4x4 matrix which can be obtained by applying tem constrains to a 16-D space.

<br>
A rigid body’s velocity, however, can be represented by three angular
velocities and three linear velocities, which together we call a spatial velocity or twist.

<br>
To represent any point in space we first need to define a frame of reference. Any representation is then done relative to this frame. The origin of this reference frame is considered as zero-point.
This reference frame is called as space frame denoted by {s}.
We also attach a body frame to our robot or the rigid body which is going to be observed, denoted by {b}.
 As the rigid body moves the frame associated with it also moves.

<br>
 <p align="center">
	<img src="Images/refframe.png" width="394" height="266"/>
</p>

The point p exists in physical space, and it does not care how we represent it. If we fix a reference frame {a}, with unit coordinate axes x̂<sub>a</sub> and ŷ<sub>a</sub> , we can represent p as p<sub>a</sub> = (1, 2). If we fix a reference frame {b} at a different location, a different orientation, and a different length scale, we can represent p as p<sub>b</sub> = (4, −2).

<br>
To describe the configuration of the planar body, only the position and
orientation of the body frame with respect to the fixed frame need to be specified.


<br>
 <p align="center">
	<img src="Images/frame.png" width="411" height="366"/>
</p>

The frame {b} in {s} is given by (P, p), and the frame {c} in {b} is given
by (Q, q). From these we can derive the frame {c} in {s}, described by (R, r). The
numerical values of the vectors p, q, and r and the coordinate-axis directions of the
three frames are evident from the grid of unit squares.

<br>

ψ is the amount of rotation required to convert x̂<sub>b</sub> to x̂<sub>c</sub>.

<br>
point p which is the origin of frame {b} can be represented as column vector,

$$
p = \begin{bmatrix}
    p_{x}  \\
    p_{y}
\end{bmatrix}
$$

this is nothing but translation matrix for frame {b}

<br>
the two vectors x̂<sub>c</sub> and ŷ<sub>b</sub> can be represented as,

$$
P = \begin{bmatrix}
    x̂_{b}  &
    ŷ_{b}
\end{bmatrix}
= \begin{bmatrix}
    x̂_{s}.cos(\theta) & -x̂_s.sin(\theta) \\
    p_{s}.sin(\theta) & ŷ_s.cos(\theta)
\end{bmatrix}
$$

Matrix P is called as **Rotation matrix**

Together, the pair (P, p) provides a description of the orientation and position of {b} relative to {s}.


## Rotation Matrix

## Angular Velocities

## Exponential Coordinate Representation

## Homogenous Transformation Matrix

## Twist

## Screw

## Exponential Coordinates

## Wrench

**Moment** or **torque** of a force *f<sub>a</sub>* acting on a rigid body at point r represented as *r<sub>a</sub>* in frame {a} is:

$$
m_a = r_a * f_a
$$

Just as with twists, we can merge the moment and force into a single six- dimensional spatial force, or wrench, expressed in the {a} frame, Fa :

$$
F_a = \begin{bmatrix}
    m_{a}  \\
    f_{a}
\end{bmatrix}
∈ ℝ^6
$$

Wrenches can be transformed from one frame to other using the fact that the power generated (dissipated) by an  wrench-twist pair must be same regardless of the frame in which they are represented.

$$
F_b = {Ad_{T_{ab}}}^{T} (F_a) = 
\begin{bmatrix}
    {Ad_{T_{ab}}}  \\
\end{bmatrix}^T
F_a
$$

$$
F_a = {Ad_{T_{ba}}}^{T} (F_b) = 
\begin{bmatrix}
    {Ad_{T_{ba}}}  \\
\end{bmatrix}^T
F_b
$$