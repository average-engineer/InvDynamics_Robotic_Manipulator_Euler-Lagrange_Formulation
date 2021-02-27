%**************************************************************************
%************** FUNCTION TO CALCULATE THE ARM MATRIX **********************
%**************************************************************************

function [A,flag] = ArmMatrix(A, InputFile)

global NJ NF DOF theta 

fid = fopen(InputFile,'r');
if(fid == -1 ) display('File Not Found !!!'); return; end

% INPUT VARIBLES
NJ = fscanf(fid,'%d',1);
DOF = fscanf(fid,'%d',1);
NF = NJ + 2;

if(NJ<1) display('Wrong Input !!! Please Check the No. of Joints.');return; end

alpha = zeros(NJ,1);    % alpha(k)  =   Twist Angle (Angle b/w Z(k-1) & Z(k) along X(k))
a = zeros(NJ,1);        % a(k)      =   Link Length (Distance from Z(k-1) to Z(k) along X(k))
theta = zeros(NJ,1);    % theta(k)  =   Joint Angle (Angle b/w X(k-1) & X(k) along Z(k-1)
d = zeros(NJ,1);        % d(k)      =   Joint Offset(Distance from X(k-1) to X(k) along Z(k-1)
flag = zeros(NJ,1);     % flag(k)   =   Decision about the Joint Type (Rotational(flag=1) or Prismatic (flag=0)) 
dQ_dt = zeros(NJ,1);    % dQ_dt(k)  =   Actuator Velocity

for k=1:NJ
    alpha(k) = fscanf(fid,'%f',1)*pi/180;
    a(k) = fscanf(fid,'%f',1);
    d(k) = fscanf(fid,'%f',1);
    theta(k) = fscanf(fid,'%f',1)*pi/180;
    dQ_dt(k) = fscanf(fid,'%f',1);
    flag(k) = fscanf(fid,'%d',1);
end


% CALCULATE THE ARM MATRIX
for k = 1:NJ
    
    if(flag(k) == 0)
    
          A(:,:,k) = [ cos(theta(k))   -cos(alpha(k))*sin(theta(k))     sin(alpha(k))*sin(theta(k))     0; 
                       sin(theta(k))    cos(alpha(k))*cos(theta(k))    -sin(alpha(k))*cos(theta(k))     0; 
                         0              sin(alpha(k))                   cos(alpha(k))                   d(k);   
                         0              0                                0                              1]  ;
          
          continue;
    end
    
    A(:,:,k) =   [ cos(theta(k))   -cos(alpha(k))*sin(theta(k))     sin(alpha(k))*sin(theta(k))     a(k)*cos(theta(k)); 
                   sin(theta(k))    cos(alpha(k))*cos(theta(k))    -sin(alpha(k))*cos(theta(k))     a(k)*sin(theta(k)); 
                     0              sin(alpha(k))                   cos(alpha(k))                   d(k)              ;   
                     0              0                                0                              1                ];

end  

