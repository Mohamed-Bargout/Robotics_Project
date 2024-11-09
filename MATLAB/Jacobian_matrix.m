function jacobian = Jacobian_matrix(q)
%q is a column vector

X = Forward_kinematics_func();


%extract x,y,z from T matrix
x = X(1);
y = X(2);
z = X(3);

 
for i = 1:4
    dxdq(i) = diff(x,1,q(i));
    dydq(i) = diff(y,1,q(i));
    dzdq(i) = diff(z,1,q(i)); 
end

jacobian = [dxdq;dydq;dzdq];
end

