function T = Forward_kinematics_func()

    syms q1 q2 q3 q4

    l1 = 65.73;   %mm to m
    l3 = 145.66;  %mm to m
    l4z = -13.5;  %mm to m
    l4x = 143.55; %mm to m
    
    % DH Convention Table
    DH = [
         q1         ,l1  ,0   ,pi/2;
        -(pi/2-q2)  ,0   ,0   ,-pi/2;
        -(pi/2-q3)  ,l3  ,0   ,-pi/2;
        -(pi/2-q4)  ,l4z ,l4x , 0;
    ];

    T = eye(4);
    % Compute the transformation matrix for each joint and multiply them
    for i = 1:size(DH, 1)
        theta_i = DH(i, 1);
        d_i = DH(i, 2);
        a_i = DH(i, 3);
        alpha_i = DH(i, 4);

        % Define the individual transformation matrix
        A = transformation_func(theta_i, d_i, a_i, alpha_i);

        % Multiply the transformation matrix to accumulate
        T = T * A;
    end
    % Extract X, Y, Z displacements from the transformation matrix
    X = T(1, 4);
    Y = T(2, 4);
    Z = T(3, 4);
    
    % Return the column vector [X; Y; Z]
    T = [X; Y; Z];

end
