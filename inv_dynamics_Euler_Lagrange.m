%**************************************************************************
%************* INVERSE DYNAMICS (3D) of Any DOF's ROBOT  ************
%****************EULER-LAGRANGE FORMULATION**************************
% WHERE
% NJ = Number of Joints 
% NF = Number of Frames
% DOF = Degree of Freedom of the Cartesian Space
% flag = Decision about the Joint Type 
% For Rotary Joint (Flag=1),
% For Prismatic Joint (Flag=0)
%**************************************************************************
% current computation time = 0.0948 seconds
% Ashutosh Mukherjee
%% SETTINGS
clc
clearvars
close all
format short

% GLOBAL VARIABLES
global NJ

NJ = 3;

% type of joint
flag = zeros(NJ,1);
for i = 1:NJ
    if i ~= 3
        flag(i) = 1;%revolute
    end
end
% link masses
for i = 1:NJ
    if i == 1 || i == 2
    link_masses(i) = 1;
    elseif i == 3
        link_masses(i) = 0.5;
    end
end

% link widths
w = zeros(NJ,1);

% link lengths
link_length = zeros(NJ,1);
for i = 1:NJ
link_length(i) = 1;
end

% height of links
H = zeros(NJ,1);

% position of link i COM wrt to ith frame (distal)
for i = 1:NJ
    if flag(i) == 1
    r{i} = [-link_length(i)/2;0;0;1];
    else
        r{i} = [0;0;-link_length(i)/2;1];
    end
end

% inertias
% Inertias computed wrt the distal joints
for i = 1:NJ
    if flag(i) == 1
        Ixx(i) = (link_masses(i)*(w(i).^2 + H(i).^2)/12);
        Iyy(i) = (link_masses(i)*(4*(link_length(i).^2) + w(i).^2)/12);
        Izz(i) = (link_masses(i)*(4*(link_length(i).^2) + H(i).^2)/3);
        Ixy(i) = -link_masses(i)*w(i)*link_length(i)/4 ;
        Ixz(i) = -link_masses(i)*w(i)*H(i)/4;
        Iyz(i) = -link_masses(i)*link_length(i)*H(i)/4;
    else
        Izz(i) = (link_masses(i)*(w(i).^2 + H(i).^2)/3);
        Iyy(i) = (link_masses(i)*(link_length(i).^2 + w(i).^2)/3);
        Ixx(i) = (link_masses(i)*(link_length(i).^2 + H(i).^2)/3);
        Ixy(i) = -link_masses(i)*w(i)*link_length(i)/4 ;
        Ixz(i) = -link_masses(i)*w(i)*H(i)/4;
        Iyz(i) = -link_masses(i)*link_length(i)*H(i)/4;
    end
end

% velocities
theta_dot = [0.5;0.25;0.125];
theta_dot_dot = ones(NJ,1);

% Acceleration due to gravity matrix
gravity_acc = zeros(1,4);
gravity_acc(3) = -9.81;
% DATA FILES    
inputFileDHP = 'dhParamthreeAxisScara.txt';

% MEMORY ALLOCATION
A = zeros(4,4,NJ);% arm matrix

%% FUNCTION CALLS
% Calculate homogeneous tranforms of each frame 
tic
[A] = ArmMatrix(A,inputFileDHP);

% inertia matrix
J = inertia_matrix(Ixx,Iyy,Izz,Ixy,Ixz,Iyz,link_masses,r,NJ);
% matrices for effect of the movement of one joint on other joint (Uij)
U = U_matrix(A,NJ,flag);
% matrices for effect of the movement of two joints on another joint (Uijk)
U1 = U1_matrix(A,NJ,flag);
% dynamic coeffient matrices
% inertial acceleration based matrix
D = inertia_acceleration_matrix(U,J,NJ);
% coroilis and centrifugal force matrix
h = centri_cor_force_matrix(U,U1,J,NJ,theta_dot);
% gravity loading based matrix
c = gravity_loading_matrix(link_masses,gravity_acc,U,r,NJ);
% generalized torque matrix at each time instant
torque = D*theta_dot_dot + h' + c'
computing_time = toc;