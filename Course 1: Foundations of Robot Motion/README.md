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
spatial mechanisms), fi be the number of freedoms provided by joint i, and ci be
the number of constraints provided by joint i, where fi + ci = m for all i. Then Degree of Freedom (DOF) is given by, 
<p align="center">
	<img src="Images/grubler.png" width="282" height="106"/>
</p>

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

<br>
Holonomic constrains reduce the C-space and thus the DOF of the robot.
<br>
Non-holonomic constraints do not limit the C-space dimension but limit the motion of the robot (e.g. translation motion of a car).
<br>

**Velocities are not the time derivatives of coordinates.**

## Task Space and Workspace

## Rigid Body Motion

## Rotation Matrix

## Angular Velocities

## Exponential Coordinate Representation

## Homogenous Transformation Matrix

## Twist

## Screw

## Exponential Coordinates

## Wrench