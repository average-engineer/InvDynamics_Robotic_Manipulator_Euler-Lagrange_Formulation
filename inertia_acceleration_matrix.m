function [D] = inertia_acceleration_matrix(U,J,NJ)

    for i = 1:NJ
        for k = 1:NJ
          D(i,k) = 0;  
            for j = max(i,k):NJ
                D_1 = trace(U{j,k}*J{j}*U{j,i}');
                D(i,k) = D(i,k) + D_1;
            end
        end
    end
end