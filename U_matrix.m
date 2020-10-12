function [U] = U_matrix(A,NJ,flag)
    for i = 1:NJ
        for j = 1:NJ
            if flag(j) == 1;
                %Q matrix for revolute joint
                Q = [0,-1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];
            elseif flag(j) == 0;
                %Q matrix for prismatic joint
                Q = [0,0,0,0;0,0,0,0;0,0,0,1;0,0,0,0];
            end
            if j>i
                U{i,j} = zeros(4,4);
            else
                T1 = T_Concat(A,j-1,0);
                T2 = T_Concat(A,i,j-1);
                U{i,j} = T1*Q*T2;
            end
        end
    end
end