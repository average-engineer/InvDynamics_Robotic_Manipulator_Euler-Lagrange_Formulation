# InvDynamics_Robotic_Manipulator_Euler-Lagrange_Formulation
Inverse dynamics of a robotic manipulator of any DOF using Lagrange Euler Dynamic Formulation (Energy Method). The inputs are the joint space variables (joint position, velocity and accelerations) and the outputs are the joint torques/forces.
The code utilizes the distal DH parameters of the manipulator. As an example, the 3-axis SCARA robot has been taken with the all the links assumed to be thin cylinders and having zero cross inertias (Ixy = Iyz = Izx = 0).
There is only one input file: a text file (`dhParamthreeAxisScara.txt`) which takes the distal DH parameters of the manipulator from the user and the joint velocities and accelerations also need to be input.

Explanation of the scripts included in the repository:

1). `ArmMatrix.m` calculates the homogeneous transformation matrices from one frame to the adjacent frame while `T_Concat.m` calculates the concatenated matrix from a starting frame to an ending frame. 

2). `inv_dynamics_Euler_Lagrange.m` is the main script which needs to be executed to compute the joint torques. Since this is an inverse dynamics code, the outputs will be the torques (in case of revolute joints) and forces (in case of prismatic joints) along the joint axes. The euler lagrange formulation is done in the matrix form, where we need to compute the inertial acceleration matrix, the centrifugal and coriolis force matrix and the gravity loading matrix.

3). `inertia_acceleration_matrix.m` is a function which calculates the inertial acceleration matrix. Each term of this matrix basically signifies the reaction torque/force on a particular joint/link due to the acceleration of another joint/link. This matrix is always symmetric and positive definite.

4). `centri_cor_force_matrix.m` is a function which calculates the centrifugal and coriolis force matrix. Each term of this matrix calculates the centrifugal force and coriolis force generated on one joint/link due to the velocity of other two joints/links.

5). `gravity_loading_matrix.m` is a function which calculates the gravity loading matrix, whose each term corresponds to the potential energy considerations of each link due to gravity. 

6). `U1_matrix.m` is a function which computes the effect on the position and orientation of a particular link due to the motion of other 2 links. This is basically a matrix. The function basically computes all such possible matrices (for eg. the motion of the 2nd and 3rd joint in the manipulator won't have any effect on the position and orientation of the base joint, instead it would be the other way around) and groups them in a tensor (cell matrix) and returns the cell matrix when called in the main script.

7). `U_matrix.m` is a function which computes the effect of the motion on 1 link/joint on the position and orientation of another link. It computes the matrix in the same way as `U1_matrix.m`.

8). `inertia_matrix.m` is a function which computes the pseudo-inertia matrix/tensor of each link in the manipulator.


