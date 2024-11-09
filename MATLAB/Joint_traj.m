function Joint_Space = Joint_traj(q0, qf, qdot0, qdotf, Tf, Ts)

% q = C0 + C1.t + C2.t^2 + C3.t^3;

num_joints = 4;

Joint_space = zeros(4,num_joints);
t = 0:Ts:Tf;

for i = 1:num_joints
    %  q1   q2   q3  q4
    % [C0 , C0 , C0, C0;
    %  C1 , C1 , C1 , C1;
    %  C2 , C2 , C2 , C2;
    %  C3 , C3 , C3 , C3]

    C0 = q0(i);
    C1 = qdot0(i);
    
    A = [Tf^2, Tf^3;
        2*Tf, 3*Tf^2];
    B = [qf(i)-C0-C1*Tf;qdotf(i) - C1;];
    
    C = linsolve(A, B);
    
    C2 = C(1);
    C3 = C(2);
    
    X = [C0;C1;C2;C3];

    Joint_space(:,i) = X;

end

% disp(Joint_space);

q1 = Joint_space(1,1) + Joint_space(2,1).*t + Joint_space(3,1)*t.^2 + Joint_space(4,1).*t.^3;
q2 = Joint_space(1,2) + Joint_space(2,2).*t + Joint_space(3,2)*t.^2 + Joint_space(4,2).*t.^3;
q3 = Joint_space(1,3) + Joint_space(2,3).*t + Joint_space(3,3)*t.^2 + Joint_space(4,3).*t.^3;
q4 = Joint_space(1,4) + Joint_space(2,4).*t + Joint_space(3,4)*t.^2 + Joint_space(4,4).*t.^3;



Joint_Space = [q1;...
               q2;...
               q3;...
               q4];

end

