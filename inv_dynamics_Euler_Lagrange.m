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
%current computation time = 10.3 minutes (618 seconds)
%% SETTINGS
clc
clear all
close all
format bank

% GLOBAL VARIABLES
global NJ NF DOF 

NJ = 3;

%type of joint
flag = zeros(NJ,1);
for i = 1:NJ
    if i ~= 3
        flag(i) = 1;%revolute
    end
end
%link masses
for i = 1:NJ
    if i == 1 || i == 2
    link_masses(i) = 1;
    elseif i == 3
        link_masses(i) = 0.5;
    end
end

%link widths
w = zeros(NJ,1);

%link lengths
link_length = zeros(NJ,1);
for i = 1:NJ
link_length(i) = 1;
end

%height of links
H = zeros(NJ,1);
% for i = 1:NJ
%     if i == 3
%         H(i) = 0.1;
%     elseif i == 4
%         H(i) = 0.2;
%     end
% end

%position of link i COM wrt to ith frame (right)
for i = 1:NJ
    if flag(i) == 1
    r{i} = [-link_length(i)/2;0;0;1];
    else
        r{i} = [0;0;-link_length(i)/2;1];
    end
end

%inertias
for i = 1:NJ
    if flag(i) == 1
    Ixx(i) = (link_masses(i)*(w(i).^2 + H(i).^2)/3);
    Iyy(i) = (link_masses(i)*(link_length(i).^2 + w(i).^2)/3);
    Izz(i) = (link_masses(i)*(link_length(i).^2 + H(i).^2)/3);
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

%velocities
theta_dot = [0.5;0.25;0.125];
theta_dot_dot = ones(NJ,1);

%Acceleration due to gravity matrix
gravity_acc = zeros(1,4);
gravity_acc(3) = -9.81;
%DATA FILES    
inputFileDHP = 'dhParamthreeAxisScara.txt';

%MEMORY ALLOCATION
A = zeros(4,4,NJ);% arm matrix

% %% FUNCTION CALLS
% Calculate homogeneous tranforms of each frame 
tic
[A] = ArmMatrix(A,inputFileDHP);
% 
% % Calculate the tip to base tip to base concatenated matrix
% tip_frame = 2; % frame at toe tip
% base_frame = 0; % frame at base or pelvis
% 
% A1 = T_Concat(A,tip_frame,base_frame); % Here  

% Plot the final concated matrix 
% A1

% % If you want to see the transformation martix of 8th frame
% A(:,:,8);  

% % If you want to calculate the matric of ankle tip to Knee (extension)
% % See fig in the paper, knee extension belongs to frame 5
% A1 = T_Concat(A,tip_frame,5);  
% 
% for kk = 1:length(impt_vars.sim_time(1:end-1))
%     A1{kk}(1,4) = A1{kk}(1,4) + impt_vars.r_asis_x(kk);
%     A1{kk}(2,4) = A1{kk}(2,4) + impt_vars.r_asis_y(kk);
%     A1{kk}(3,4) = A1{kk}(3,4) + impt_vars.r_asis_z(kk);
%     x_data_toe(kk) = A1{kk}(1,4);
%     y_data_toe(kk) = A1{kk}(2,4);
%     z_data_toe(kk) = A1{kk}(3,4);
% end

% figure(1)
% hold on
% aa = plot(impt_vars.sim_time,impt_vars.r_met_x,'*')
% bb = plot(impt_vars.sim_time,x_data_toe)
% legend([aa,bb],'Marker Data','MATLAB result in SP')
% title('Right Toe (r-met marker) X - trajectory')
% xlabel('Time (s)')
% ylabel('X-coordinate of Toe Marker')
% figure(2)
% hold on
% cc = plot(impt_vars.sim_time,impt_vars.r_met_y,'*')
% dd = plot(impt_vars.sim_time,y_data_toe)
% legend([cc,dd],'Marker Data','MATLAB result in SP')     
% title('Right Toe (r-met marker) Y - trajectory')
% xlabel('Time (s)')
% ylabel('Y-coordinate of Toe Marker')
% figure(3)
% hold on
% ee = plot(impt_vars.sim_time,impt_vars.l_met_z,'*')
% ff = plot(impt_vars.sim_time,z_data_toe)
% legend([ee,ff],'Marker Data','MATLAB result in SP')
% title('Left Toe (l-met marker) Z - trajectory')
% xlabel('Time (s)')
% ylabel('Z-coordinate of Toe Marker')

%inertia matrix
J = inertia_matrix(Ixx,Iyy,Izz,Ixy,Ixz,Iyz,link_masses,r,NJ);
%matrices for effect of the movement of one joint on other joint (Uij)
U = U_matrix(A,NJ,flag);
%matrices for effect of the movement of two joints on another joint (Uijk)
U1 = U1_matrix(A,NJ,flag);
%dynamic coeffient matrices
%inertial acceleration based matrix
D = inertia_acceleration_matrix(U,J,NJ);
%coroilis and centrifugal force matrix
h = centri_cor_force_matrix(U,U1,J,NJ,theta_dot);
%gravity loading based matrix
c = gravity_loading_matrix(link_masses,gravity_acc,U,r,NJ);
%generalized torque matrix at each time instant
torque = D*theta_dot_dot + h' + c'
computing_time = toc;