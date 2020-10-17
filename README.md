# InvDynamics_Robotic_Manipulator_Euler-Lagrange_Formulation
Inverse dynamics of a robotic manipulator of any DOF using Lagrange Euler Dynamic Formulation (Energy Method). The inputs are the joint space variables (joint position, velocity and accelerations) and the outputs are the joint torques/forces.
The code utilizes the distal DH parameters of the manipulator. As an example, the 3-axis SCARA robot has been taken with the all the links assumed to be thin cylinders and having zero cross inertias (Ixy = Iyz = Izx = 0).
There is only one input file: a text file which takes the distal DH parameters of the manipulator from the user and the joint velocities and accelerations also need to be input.

Explanation of the scripts included in the repository:


