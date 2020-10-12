function [J] = inertia_matrix(Ixx,Iyy,Izz,Ixy,Ixz,Iyz,m,r,NJ)

for i = 1:NJ
    %inertia matrix for each link i
    J{i} = [(-Ixx(i) + Iyy(i) + Izz(i))/2,Ixy(i),Ixz(i),m(i)*r{i}(1);
        Ixy(i),(Ixx(i) - Iyy(i) + Izz(i))/2,Iyz(i),m(i)*r{i}(2);
        Ixz(i),Iyz(i),(Ixx(i) + Iyy(i) - Izz(i))/2,m(i)*r{i}(3);
        m(i)*r{i}(1),m(i)*r{i}(2),m(i)*r{i}(3),m(i)];
end
end