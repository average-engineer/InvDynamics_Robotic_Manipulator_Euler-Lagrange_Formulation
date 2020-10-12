function [U1] = U1_matrix(A,NJ,flag)
    for i = 1:NJ
        for j = 1:NJ
            if flag(j) == 1
                Qj = [0,-1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];;
            elseif flag(j) == 0
                Qj = [0,0,0,0;0,0,0,0;0,0,0,1;0,0,0,0];
            end
            for k = 1:NJ
                if flag(k) == 1
                    Qk = [0,-1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];;
                elseif flag(k) == 0
                    Qk = [0,0,0,0;0,0,0,0;0,0,0,1;0,0,0,0];
                end
                if j<=k&&k<=i
                    T1 = T_Concat(A,j-1,0);
                    T2 = T_Concat(A,k-1,j-1);
                    T3 = T_Concat(A,i,k-1);
                    U1{i}{j,k} = T1*Qj*T2*Qk*T3;
                elseif k<=j&&j<=i    
                    T4 = T_Concat(A,k-1,0);
                    T5 = T_Concat(A,j-1,k-1);
                    T6 = T_Concat(A,i,j-1);
                    U1{i}{j,k} = T4*Qk*T5*Qj*T6;
                elseif j>i||k>i
                    U1{i}{j,k} = zeros(4,4);
                end
            end
        end
    end
end
