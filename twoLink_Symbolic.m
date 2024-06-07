%% Symbolic Derivation of 2-Link Manipulator using Euler-Lagrange Inverse Dynamics
% Ashutosh Mukherjee
% Requirement: MATLAB Symbolic Math Toolbox
%% Settings
clc
clearvars
close all
sympref('FloatingPointOutput', false) % to display fractions instead of decimals
%% Assumptions
% Links are assumed to be thin cylinders
% Symmetry assumed such that cross-inertia terms are zero
%% Dynamic Parameters
% Number of Joints
NJ = 2;
% Link Masses
syms m1 m2 real
link_masses = [m1;m2];
% Link Lengths
syms l1 l2 real
link_lengths = [l1;l2];
% Distance of Link COM from distal joint
for i = 1:NJ
    r{i} = [-link_lengths(i)/2;0;0;1];
end
% Link Inertias
for i = 1:NJ
    Ixx(i) = 0;
    Iyy(i) = link_masses(i)*(link_lengths(i)^2)/3;
    Izz(i) = Iyy(i);
    Ixy(i) = 0;
    Ixz(i) = 0;
    Iyz(i) = 0;
end

% Acceleration due to gravity vector
syms g real; % Acceleration due to gravity
gravity_acc = [0,-g,0,0];

% Joint Flags (1: revolute, 0: prismatic)
% All are revolute
flags = ones(NJ,1);
%% Joint coordinates
syms q1 q2 q1_d q2_d q1_dd q2_dd real
theta = [q1; q2];
theta_dot = [q1_d;q2_d];
theta_dot_dot = [q1_dd;q2_dd];
%% Distal DH parameters
% Angles between the Z-axes of joints
alpha = zeros(NJ,1);
% Link Lengths
a = link_lengths;
% Offset between joint frames (along X)
d = zeros(NJ,1);
%% Arm Matrix
for k = 1:NJ
    % All joints are revolute
    A(:,:,k) =   [ cos(theta(k))   -cos(alpha(k))*sin(theta(k))     sin(alpha(k))*sin(theta(k))     a(k)*cos(theta(k));
        sin(theta(k))    cos(alpha(k))*cos(theta(k))    -sin(alpha(k))*cos(theta(k))     a(k)*sin(theta(k));
        0              sin(alpha(k))                   cos(alpha(k))                   d(k)              ;
        0              0                                0                              1                ];

end
%% Inertia Matrix
J = inertia_matrix(Ixx,Iyy,Izz,Ixy,Ixz,Iyz,link_masses,r,NJ);
%% Matrices for effect of the movement of one joint on other joint (Uij)
U = U_matrix(A,NJ,flag);
%% Matrices for effect of the movement of two joints on another joint (Uijk)
U1 = U1_matrix(A,NJ,flag);
%% Inertial acceleration based matrix
D = inertia_acceleration_matrix_symbolic(U,J,NJ);
%% Coroilis and centrifugal force matrix
h = centri_cor_force_matrix_symbolic(U,U1,J,NJ,theta_dot);
%% Gravity loading based matrix
c = gravity_loading_matrix_symbolic(link_masses,gravity_acc,U,r,NJ);
%% Generalized torque vector
torque = D*theta_dot_dot + h' + c';
torque = simplify(torque)