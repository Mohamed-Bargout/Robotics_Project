function T = transformation_func(theta, d, a, alpha)
    % Define symbolic variables for DH parameters
    %syms theta_i d_i a_i alpha_i
    
    % Define the individual transformation matrix
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha) , a*cos(theta);
         sin(theta), cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta);
         0         , sin(alpha)            ,cos(alpha)             , d           ;
         0         , 0                     ,0                      , 1];
    
    % Return the individual transformation matrix
    T = A;
end