
% CALCULATE THE CONCATENATED MATRIX
function T = T_Concat(A,tip,base)

T = eye(4);
if((tip == 0 && base == 0)||(tip == base))
    T = eye(4);
elseif (tip > base)  
    for n =  tip : -1 : base+1
        T = A(:,:,n)*T;
    end  
end
end
