function [c] = gravity_loading_matrix_symbolic(m,g,U,r,NJ)
    for i = 1:NJ
        c(i) = sym(0);
        for j = i:NJ
            c1 = -m(j)*g*U{j,i}*r{j};
            c(i) = c(i) + c1;
        end
    end
end