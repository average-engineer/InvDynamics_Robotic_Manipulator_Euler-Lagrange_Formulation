function [h_star] = centri_cor_force_matrix_symbolic(U,U1,J,NJ,theta_dot)

    for i = 1:NJ
        for k = 1:NJ
            for m = 1:NJ
                h{i}(k,m) = sym(0);
                for j = max([i,k,m]):NJ
                    h1 = trace(U1{j}{k,m}*J{j}*(U{j,i}'));
                    h{i}(k,m) = h{i}(k,m) + h1;
                end
            end
        end
    end
    for i = 1:NJ
        h_star(i) = theta_dot'*h{i}*theta_dot;
    end
end